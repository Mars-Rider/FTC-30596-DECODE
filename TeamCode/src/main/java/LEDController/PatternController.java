package LEDController;
public class PatternController {
    public enum Pattern {
        SOLID(1), BLINK(2), LARSON(3), BREATHE(4),
        //GRADIENT_STATIC(5), GRADIENT_MOVING(6),
        PINGPONG(7),
        //FLICKER(8), IN DEVELOPMENT
        VALUE(8),
        HEARTBEAT(9), RADAR(10), SLIDER(11), BOUNCING_SLIDER(12), WAVE(13);

        public final int id;
        Pattern(int id){ this.id=id; }
    }

    Pattern patternVal = Pattern.SOLID;

    int trailLength = 4;
    int sourceCenter = 0;
    byte sourceLength = 5; //How big the length is of the thing moving or being
    boolean gradient = false;
    boolean moving = false;
    int period = 500;
    boolean direction = true; //Right is true

    private final LEDChannel parent;
    PatternController(LEDChannel parent){ this.parent = parent;}

    public void setBreathe(){ patternVal= Pattern.BREATHE; update(); }
    public void setBlink(){ setBlink(500);}//500 ms period
    public void setBlink(int delay){ period = delay; patternVal= Pattern.BLINK; update(); }
    public void setScanner(){ setScanner(4, sourceLength, true); }
    public void setScanner(int trail, int length, boolean movingRight){ trailLength=trail; patternVal= Pattern.LARSON; sourceLength = (byte)parent.clamp(length, 1, 256); direction = movingRight; update(); }
    public void setRadar(){ setRadar(15,3); }
    public void setRadar(int center,int trail){ sourceCenter=center; trailLength=trail; patternVal= Pattern.RADAR; update(); }
    public void setPingPong(){ setPingPong(4); }
    public void setPingPong(int trail){ trailLength=trail; patternVal= Pattern.PINGPONG; update(); }
    public void setHeartbeat(int delay){ period=delay; patternVal= Pattern.HEARTBEAT; update(); }
    public void setSolid(){patternVal= Pattern.SOLID; update(); }

    //IN DEVELOPMENT
        /*
        public void setFlicker(){ setFlicker(0,360); }
        public void setFlicker(int minHue,int maxHue){ twinkleHueMin=minHue; twinkleHueMax=maxHue; patternVal=Pattern.FLICKER; autoUpdate(); }*/

    private void update(){
        parent.update();
    }
}
