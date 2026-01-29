package LEDController;
import java.util.Arrays;

public class LEDChannel {
    // Controllers
    public final ColorController color = new ColorController(this);
    public final PatternController pattern = new PatternController(this);

    // Core properties
    LEDController.Strip type;
    private int brightness = 255;

    private final LEDController parent;

    LEDChannel(LEDController parent, LEDController.Strip s){ this.parent = parent; type = s;}

    private byte[] lastData = new byte[20];
    void update(){
        byte[] data=this.toPacket();
        if(!Arrays.equals(data,lastData)){
            lastData=data.clone();
            parent.device.write(0x00,data);
        }
    }//Sends data if data is new

    //Individual LED Control - IN DEVELOPMENT
        /*public void setLED(int ledIndex,int r,int g,int b,int brightness){
            int[] hs = rgbToHs(r,g,b);

            setLED(ledIndex,hs[0],hs[1], brightness);
        }*/
        /*public void setLED(int ledIndex,int h,int s,int brightness){
            if(ledIndex<0 || ledIndex>=numLEDs) return;
            byte[] packet=new byte[10];
            packet[0]=(byte)getChannelIndex();
            packet[1]=(byte)((ledIndex>>8)&0xFF);
            packet[2]=(byte)(ledIndex&0xFF);
            packet[3] = (byte)((h >> 8) & 0xFF);
            packet[4] = (byte)(h & 0xFF);
            packet[5] = (byte)s;
            packet[6] = (byte)brightness;//brightness
            packet[9]=0;//Pattern to individual led
            device.write(0x00,packet);
        }*/

    int getChannelIndex(){
        return parent.channels.indexOf(this);
    }

    public void setBrightness(double b){
        brightness=(int)clamp(b,0,100)*255;
        update();
    }

    //IN DEVELOPMENT
        /*
        public void setNumLEDs(int count){ numLEDs=Math.min(count,1000); autoUpdate(); } //Not Needed as num LEDs is set at start and not looked at after*/

    double clamp(double val,double min,double max){ return Math.max(min,Math.min(max,val)); }
    int clamp(int val){ return Math.max(0,Math.min(255,val)); }
    double clamp01(double val){ return Math.max(0,Math.min(1,val)); }

    // Packet Builder
    private byte[] toPacket() {
        byte[] packet = new byte[22]; // 16 bytes: numLEDs, RGB, brightness, pattern id + params MAX OF 30

        packet[0] = (byte)getChannelIndex();

        // Compute hsv if using VALUE
        int h = color.colorVal.h;
        int s = color.colorVal.s;
        if(color.colorVal == ColorController.Color.VALUE){
            double min = color.valueStuff[0], max = color.valueStuff[1], val = color.valueStuff[2];
            double t = (max == min ? 0 : (val - min) / (max - min));
            t = clamp01(t);
            double hue = color.normalizeHue(color.valueStuff[3] + t * (color.valueStuff[4] - color.valueStuff[3]));
            h = (int)hue; s = 255;
        }

        // Base color
        packet[1] = (byte)((h >> 8) & 0xFF);
        packet[2] = (byte)(h & 0xFF);
        packet[3] = (byte)s;
        packet[4] = (byte)brightness;//brightness

        //Color Parameters
        packet[5] = (byte)(color.colorVal == ColorController.Color.RAINBOW ? 1 : 0);
        packet[6] = (byte)((color.period >> 8) & 0xFF);
        packet[7] = (byte)(color.period);
        packet[8] = (byte)((color.hueMin >> 8) & 0xFF);  // gradient moving flag
        packet[9] = (byte)(color.hueMin & 0xFF);  // gradient moving flag
        packet[10] = (byte)((color.hueMax >> 8) & 0xFF);  // gradient moving flag
        packet[11] = (byte)(color.hueMax & 0xFF);  // gradient moving flag

        // Pattern
        packet[12] = (byte)pattern.patternVal.id;

        // Pattern parameters
        packet[13] = (byte)pattern.trailLength;                 // e.g., Larson / PingPong
        packet[14] = (byte)((pattern.sourceCenter >> 8) & 0xFF);
        packet[15] = (byte)(pattern.sourceCenter & 0xFF);
        packet[16] = (byte)pattern.sourceLength;
        packet[17] = (byte)(pattern.gradient ? 1 : 0);  // gradient moving flag
        packet[18] = (byte)(pattern.moving ? 1 : 0);  // gradient moving flag
        packet[19] = (byte)((pattern.period >> 8) & 0xFF);    // period high byte
        packet[20] = (byte)(pattern.period & 0xFF);           // period low byte
        packet[21] = (byte)(pattern.direction ? 1 : 0);           // period low byte

        return packet;
    }
}