package LEDController;

import LEDController.PatternController;

public class ColorController {
    public enum Color {
        RED(0,255), ORANGE(30,255), YELLOW(50,255),
        GREEN(60,255), BLUE(180,255), PURPLE(250,255),
        PINK(70,255), WHITE(0,0), CUSTOM(0,0),
        VALUE(0,0), RAINBOW(0,0);

        public int h,s;
        Color(int H,int S){ this.h=H; this.s=S; }
    }

    Color colorVal = Color.WHITE;

    // Value Stuff
    double[] valueStuff = {0,1,0,0,360};
    int period = 10000; //Period for rainbow to cycle is true
    int hueMin = 10000; //Period for rainbow to cycle is true
    int hueMax = 10000; //Period for rainbow to cycle is true

    private final LEDChannel parent;
    ColorController(LEDChannel parent){ this.parent = parent;}

    double normalizeHue(double h){ return ((h % 360) + 360) % 360; }
    int[] rgbToHs(int r, int g, int b) {
        float rf = r / 255f;
        float gf = g / 255f;
        float bf = b / 255f;

        float max = Math.max(rf, Math.max(gf, bf));
        float min = Math.min(rf, Math.min(gf, bf));
        float delta = max - min;

        int h = 0, s;

        // Hue calculation
        if (delta == 0) {
            h = 0;
        } else if (max == rf) {
            h = (int)(60 * (((gf - bf) / delta) % 6));
        } else if (max == gf) {
            h = (int)(60 * (((bf - rf) / delta) + 2));
        } else if (max == bf) {
            h = (int)(60 * (((rf - gf) / delta) + 4));
        }
        if (h < 0) h += 360;

        // Saturation
        s = (max == 0) ? 0 : (int)(delta / max);

        //MAKE SURE S IS 100 NOT 255

        return new int[]{h, s}; // h in [0-360], s and v in [0-1]
    }
    
    public void setWhite(){ colorVal= Color.WHITE; update(); }
    public void setRed(){ colorVal= Color.RED; update(); }
    public void setGreen(){ colorVal= Color.GREEN; update(); }
    public void setBlue(){ colorVal= Color.BLUE; update(); }
    public void setOrange(){ colorVal= Color.ORANGE; update(); }
    public void setYellow(){ colorVal= Color.YELLOW; update(); }
    public void setPurple(){ colorVal= Color.PURPLE; update(); }
    public void setPink(){ colorVal= Color.PINK; update(); }
    public void setRGB(int r,int g,int b){
        int[] hs = rgbToHs(r,g,b);

        Color.CUSTOM.h=parent.clamp(hs[0]);
        Color.CUSTOM.s=parent.clamp(hs[1]);
        colorVal = Color.CUSTOM; update();
    }
    public void setHS(int h,double s){
        h = (int)parent.clamp(h, 0, 360);
        s = (parent.clamp(s, 0, 100)*255);

        Color.CUSTOM.h=h;
        Color.CUSTOM.s= (int) s;
        colorVal = Color.CUSTOM; update();
    }
    public void setRainbow(){setRainbow(period, false);}
    public void setRainbow(int P, boolean Moving){ colorVal= Color.RAINBOW; parent.pattern.moving=Moving; period = P; update(); }

    //Add this new gradient mechanic
    public void setGradient(){ setGradient(0,360, true, true); }//Sets a rightward moving gradient of all the hues (rainbow)
    public void setGradient(int startHue, int endhue, boolean Moving){ setGradient(startHue,endhue,Moving, true); }//Sets a rightward moving gradient of all the hues (rainbow)
    public void setGradient(int startHue,int endHue, boolean Moving, boolean movingRight){
        if(parent.pattern.patternVal != PatternController.Pattern.VALUE){//Only add a gradient on patterns it can work with
        hueMin=startHue; hueMax=endHue; parent.pattern.moving=Moving; parent.pattern.direction = movingRight; update();}
    }//Sets a rightward moving gradient of all the hues (rainbow)

    public void setValue(double input, double minVal, double maxVal, double startHue, double endHue){
        valueStuff[0]=minVal; valueStuff[1]=maxVal; valueStuff[2]=input;
        valueStuff[3]=normalizeHue(startHue); valueStuff[4]=normalizeHue(endHue);
        colorVal = Color.VALUE; update();
    }
    public void setValueRange(double minVal, double maxVal, double startHue, double endHue){
        valueStuff[0]=minVal; valueStuff[1]=maxVal;
        valueStuff[2]=parent.clamp(valueStuff[2],minVal,maxVal);
        valueStuff[3]=normalizeHue(startHue);
        valueStuff[4]=normalizeHue(endHue);
        colorVal = Color.VALUE;
        update();
    }
    public void updateValue(double input){
        valueStuff[2]=parent.clamp(input,valueStuff[0],valueStuff[1]); colorVal = Color.VALUE; update();
    }
    
    private void update(){
        parent.update();
    }
}
