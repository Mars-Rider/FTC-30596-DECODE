package org.firstinspires.ftc.teamcode.Robot.LEDs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class LEDController {

    private final I2cDeviceSynch device;
    private final List<LEDChannel> channels = new ArrayList<>();

    private enum Color {
        RED(0,100,100), ORANGE(30,100,100), YELLOW(50,100,100),
        GREEN(60,100,100), BLUE(180,100,100), PURPLE(250,100,100),
        PINK(70,100,100), WHITE(0,0,100), CUSTOM(0,0,0),
        VALUE(0,0,0), RAINBOW(0,0,0);

        public int H,S,V;
        Color(int H,int S,int V){ this.H=H; this.S=S; this.V=V; }
    }

    private enum Pattern {
        BLINK(1), LARSON(2), SOLID(3), BREATHE(4),
        GRADIENT_STATIC(5), GRADIENT_MOVING(6),
        PINGPONG(7), FLICKER(8), HEARTBEAT(9), RADAR(10);

        public final int id;
        Pattern(int id){ this.id=id; }
    }

    public class LEDChannel {

        // Controllers
        public final ColorController color = new ColorController();
        public final PatternController pattern = new PatternController();

        // Core properties
        private Color colorVal = Color.WHITE;
        private Pattern patternVal = Pattern.SOLID;
        private int brightness = 255;
        private int numLEDs = 30;

        // Value pattern
        private double[] valueStuff = {0,1,0,0,360};

        // Extra pattern params
        private int trailLength = 4;
        private int radarCenter = 0;
        private int twinkleHueMin=0, twinkleHueMax=360;
        private boolean gradientMoving = false;
        private int period = 0;
        private boolean direction = true; //Right is true

        private final LEDController parent;

        private LEDChannel(LEDController parent){ this.parent = parent; }

        private void autoUpdate(){ parent.update(); }

        private double normalizeHue(double h){ return ((h % 360) + 360) % 360; }

        private int[] hslToRgb(double h, double s, double l){
            h = normalizeHue(h);
            s = clamp01(s/100.0);
            l = clamp01(l/100.0);
            double c=(1-Math.abs(2*l-1))*s;
            double x=c*(1-Math.abs((h/60.0)%2-1));
            double m=l-c/2;
            double r1=0,g1=0,b1=0;
            if(h<60){ r1=c; g1=x; }
            else if(h<120){ r1=x; g1=c; }
            else if(h<180){ g1=c; b1=x; }
            else if(h<240){ g1=x; b1=c; }
            else if(h<300){ r1=x; b1=c; }
            else{ r1=c; b1=x; }
            return new int[]{
                    clamp((int)((r1+m)*255)),
                    clamp((int)((g1+m)*255)),
                    clamp((int)((b1+m)*255))
            };
        }

        // ---------------- Color Controller ----------------
        public class ColorController {
            public void setWhite(){ colorVal=Color.WHITE; autoUpdate(); }
            public void setRed(){ colorVal=Color.RED; autoUpdate(); }
            public void setGreen(){ colorVal=Color.GREEN; autoUpdate(); }
            public void setBlue(){ colorVal=Color.BLUE; autoUpdate(); }
            public void setOrange(){ colorVal=Color.ORANGE; autoUpdate(); }
            public void setYellow(){ colorVal=Color.YELLOW; autoUpdate(); }
            public void setPurple(){ colorVal=Color.PURPLE; autoUpdate(); }
            public void setPink(){ colorVal=Color.PINK; autoUpdate(); }
            public void setRGB(int r,int g,int b){
                //convert to hsl

                Color.CUSTOM.h=clamp(r);
                Color.CUSTOM.s=clamp(g);
                Color.CUSTOM.l=clamp(b);
                colorVal=Color.CUSTOM; autoUpdate();
            }
            public void setHSV(int h,int s,int l){
                Color.CUSTOM.h=h;
                Color.CUSTOM.s=s;
                Color.CUSTOM.v=v;
                colorVal=Color.CUSTOM; autoUpdate();
            }
            public void setRainbow(){ colorVal=Color.RAINBOW; autoUpdate(); }
            public void setValue(double input, double minVal, double maxVal, double startHue, double endHue){
                valueStuff[0]=minVal; valueStuff[1]=maxVal; valueStuff[2]=input;
                valueStuff[3]=normalizeHue(startHue); valueStuff[4]=normalizeHue(endHue);
                colorVal=Color.VALUE; autoUpdate();
            }
            public void setValueRange(double minVal, double maxVal, double startHue, double endHue){
                valueStuff[0]=minVal; valueStuff[1]=maxVal;
                valueStuff[2]=clamp(valueStuff[2],minVal,maxVal);
                valueStuff[3]=normalizeHue(startHue);
                valueStuff[4]=normalizeHue(endHue);
                colorVal=Color.VALUE; autoUpdate();
            }
            public void updateValue(double input){
                valueStuff[2]=clamp(input,valueStuff[0],valueStuff[1]); colorVal=Color.VALUE; autoUpdate();
            }
        }

        // ---------------- Pattern Controller ----------------
        public class PatternController {
            public void setBreathe(){ patternVal=Pattern.BREATHE; autoUpdate(); }
            public void setBlink(){ setBlink(500);}//500 ms period
            public void setBlink(int delay){ period = delay; patternVal=Pattern.BLINK; autoUpdate(); }
            public void setScanner(){ setScanner(4, true); }
            public void setScanner(int trail, boolean movingRight){ trailLength=trail; patternVal=Pattern.LARSON; direction = movingRight; autoUpdate(); }
            public void setRadar(){ setRadar(numLEDs/2,3); }
            public void setRadar(int center,int trail){ radarCenter=center; trailLength=trail; patternVal=Pattern.RADAR; autoUpdate(); }
            public void setGradient(){ setGradient(0,360,false, true); }
            public void setGradient(int startHue,int endHue,boolean moving, boolean movingRight){
                valueStuff[3]=startHue; valueStuff[4]=endHue; gradientMoving=moving; direction = movingRight;
                patternVal=moving? Pattern.GRADIENT_MOVING : Pattern.GRADIENT_STATIC; autoUpdate();
            }
            public void setPingPong(){ setPingPong(4); }
            public void setPingPong(int trail){ trailLength=trail; patternVal=Pattern.PINGPONG; autoUpdate(); }
            public void setFlicker(){ setFlicker(0,360); }
            public void setFlicker(int minHue,int maxHue){ twinkleHueMin=minHue; twinkleHueMax=maxHue; patternVal=Pattern.FLICKER; autoUpdate(); }
            public void setHeartbeat(int delay){ period=delay; patternVal=Pattern.HEARTBEAT; autoUpdate(); }
            public void setSolid(){patternVal=Pattern.SOLID; autoUpdate(); }
        }

        // ---------------- LED Control ----------------
        public void setLED(int ledIndex,int r,int g,int b,int brightness){
            if(ledIndex<0 || ledIndex>=numLEDs) return;
            byte[] packet=new byte[8];
            packet[0]=(byte)0xFF; packet[1]=(byte)getChannelIndex();
            packet[2]=(byte)((ledIndex>>8)&0xFF); packet[3]=(byte)(ledIndex&0xFF);
            packet[4]=(byte)clamp(r); packet[5]=(byte)clamp(g); packet[6]=(byte)clamp(b); packet[7]=(byte)clamp(brightness);
            device.write(0x00,packet);
        }
        public void setLED(int ledIndex,double h,double s,double l,int brightness){
            int[] rgb = hslToRgb(h,s,l);

            if(ledIndex<0 || ledIndex>=numLEDs) return;
            byte[] packet=new byte[8];
            packet[0]=(byte)0xFF; packet[1]=(byte)getChannelIndex();
            packet[2]=(byte)((ledIndex>>8)&0xFF); packet[3]=(byte)(ledIndex&0xFF);
            packet[4]=(byte)clamp(rgb[0]); packet[5]=(byte)clamp(rgb[1]); packet[6]=(byte)clamp(rgb[2]); packet[7]=(byte)clamp(brightness);
            device.write(0x00,packet);
        }

        private int getChannelIndex(){
            return channels.indexOf(this);
        }

        public void setBrightness(int b){ brightness=clamp(b,0,100)*255; autoUpdate(); }
        public void setNumLEDs(int count){ numLEDs=Math.min(count,1000); autoUpdate(); }

        private double clamp(double val,double min,double max){ return Math.max(min,Math.min(max,val)); }
        private int clamp(int val){ return Math.max(0,Math.min(255,val)); }
        private double clamp01(double val){ return Math.max(0,Math.min(1,val)); }

        // ---------------- Packet Builder ----------------
        private byte[] toPacket() {
            byte[] packet = new byte[17]; // 16 bytes: numLEDs, RGB, brightness, pattern id + params

            packet[0] = (byte)getChannelIndex();

            // Number of LEDs
            packet[1] = (byte)((numLEDs >> 8) & 0xFF);
            packet[2] = (byte)(numLEDs & 0xFF);

            // Compute hsv if using VALUE
            int h = colorVal.h;
            int s = colorVal.s;
            int c = colorVal.v;
            if(colorVal == Color.VALUE){
                double min = valueStuff[0], max = valueStuff[1], val = valueStuff[2];
                double t = (max == min ? 0 : (val - min) / (max - min));
                t = clamp01(t);
                double hue = normalizeHue(valueStuff[3] + t * (valueStuff[4] - valueStuff[3]));
                h = hue; s = 100; v = 100;
            }

            // Base color
            packet[3] = (byte)h;
            packet[4] = (byte)s;
            packet[5] = (byte)v;

            // Brightness
            packet[6] = (byte)brightness;

            // Pattern
            packet[7] = (byte)patternVal.id;

            // Pattern parameters
            packet[8] = (byte)trailLength;                 // e.g., Larson / PingPong
            packet[9] = (byte)((radarCenter >> 8) & 0xFF);
            packet[10] = (byte)(radarCenter & 0xFF);
            packet[11] = (byte)((twinkleHueMin >> 8) & 0xFF);
            packet[12] = (byte)twinkleHueMax;             // limit 0-255
            packet[13] = (byte)(gradientMoving ? 1 : 0);  // gradient moving flag
            packet[14] = (byte)((period >> 8) & 0xFF);    // period high byte
            packet[15] = (byte)(period & 0xFF);           // period low byte
            packet[16] = (byte)(direction ? 1 : 0);           // period low byte

            return packet;
        }

    }

    public LEDController(HardwareMap hardwareMap,String deviceName){
        device=hardwareMap.get(I2cDeviceSynch.class,deviceName); device.engage();
    }

    public LEDChannel addChannel() {
        if (channels.size() >= 5) throw new IllegalStateException("Maximum of 5 LED channels allowed");
        LEDChannel ch = new LEDChannel(this);
        channels.add(ch);
        return ch;
    }


    private byte[] lastData;
    public void update(){
        int index=0;
        for(LEDChannel ch:channels){
            byte[] data=ch.toPacket();
            if(!Arrays.equals(data,lastData)){
                lastData=data.clone();
                byte[] packet=new byte[data.length+1]; packet[0]=(byte)index++;
                System.arraycopy(data,0,packet,1,data.length);
                device.write(0x00,packet);
            }
        }
    }
}
