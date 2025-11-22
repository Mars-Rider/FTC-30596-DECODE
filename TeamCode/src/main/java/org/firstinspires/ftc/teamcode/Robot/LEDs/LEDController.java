package org.firstinspires.ftc.teamcode.Robot.LEDs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.LinkedHashMap;
import java.util.Map;

public class LEDController {

    private I2cDeviceSynch device;
    private Map<String, LEDChannel> channels = new LinkedHashMap<>();

    public enum Color {
        RED(255,0,0), ORANGE(255,165,0), YELLOW(255,255,0),
        GREEN(0,255,0), BLUE(0,0,255), PURPLE(128,0,128),
        PINK(255,20,147), WHITE(255,255,255), CUSTOM(150,150,150);

        public int r,g,b;
        Color(int r,int g,int b){ this.r=r; this.g=g; this.b=b; }
    }

    public enum Pattern { RAINBOW(1), BLINK(2), SCANNER(3), SOLID(4), BREATHE(5);
        public final int id;
        Pattern(int id){this.id=id;}
    }

    public class LEDChannel {
        private Color color = Color.WHITE;
        private Pattern pattern = Pattern.SOLID;
        private int brightness = 255;
        private int numLEDs = 30;
        private LEDController parent;

        private LEDChannel(LEDController parent){ this.parent = parent; }

        private void autoUpdate(){ parent.update(); }

        public void setColor(Color c){ color=c; autoUpdate(); }
        public void setRGB(int r,int g,int b){
            Color.CUSTOM.r=clamp(r); Color.CUSTOM.g=clamp(g); Color.CUSTOM.b=clamp(b);
            color=Color.CUSTOM; autoUpdate();
        }
        public void setHSL(double h,double s,double l){
            h=((h%360)+360)%360;
            s=clamp01(s); l=clamp01(l);
            double c=(1-Math.abs(2*l-1))*s;
            double x=c*(1-Math.abs((h/60.0)%2-1));
            double m=l-c/2;
            double r1=0,g1=0,b1=0;
            if(h<60){r1=c; g1=x; b1=0;}
            else if(h<120){r1=x; g1=c; b1=0;}
            else if(h<180){r1=0; g1=c; b1=x;}
            else if(h<240){r1=0; g1=x; b1=c;}
            else if(h<300){r1=x; g1=0; b1=c;}
            else{r1=c; g1=0; b1=x;}
            setRGB((int)((r1+m)*255),(int)((g1+m)*255),(int)((b1+m)*255));
        }
        public void setBrightness(int b){ brightness=clamp(b); autoUpdate(); }
        public void setPattern(Pattern p){ pattern=p; autoUpdate(); }
        public void setNumLEDs(int count){ numLEDs = Math.min(count, 1000); autoUpdate(); }

        private int clamp(int val){ return Math.max(0,Math.min(255,val)); }
        private double clamp01(double val){ return Math.max(0.0,Math.min(1.0,val)); }

        private byte[] toPacket(){
            // numLEDs is 2 bytes (high + low) to support >255
            byte high = (byte)((numLEDs >> 8) & 0xFF);
            byte low = (byte)(numLEDs & 0xFF);
            return new byte[]{high, low, (byte)color.r,(byte)color.g,(byte)color.b,(byte)brightness,(byte)pattern.id};
        }
    }

    public LEDController(HardwareMap hardwareMap, String deviceName){
        device = hardwareMap.get(I2cDeviceSynch.class, deviceName);
        device.engage();
    }

    public LEDChannel addChannel(String name){
        LEDChannel ch = new LEDChannel(this);
        channels.put(name,ch);
        return ch;
    }

    public void update(){
        int index=0;
        for(LEDChannel ch : channels.values()){
            byte[] data = ch.toPacket();
            byte[] packet = new byte[data.length+1];
            packet[0]=(byte)index++;
            System.arraycopy(data,0,packet,1,data.length);
            device.write(0x00,packet);
        }
    }
}
