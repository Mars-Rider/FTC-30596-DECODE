#include <Wire.h>
#include <FastLED.h>

#define MAX_STRIPS 6
#define MAX_LEDS 1000

const int ledPins[MAX_STRIPS] = {D1,D2,D3,D4,D5,D6};
const byte I2C_ADDRESS = 0x08;

enum Pattern { RAINBOW=1, BLINK, SCANNER, SOLID, BREATHE };

struct LEDChannel {
    CRGB* leds;
    byte r,g,b;
    byte brightness;
    byte pattern;
    byte lastR,lastG,lastB,lastBrightness,lastPattern;
    int numLEDs;
    unsigned long lastUpdate;
    int scannerPos;
    bool blinkState;
};

LEDChannel strips[MAX_STRIPS];
int numStrips = MAX_STRIPS;

void setup(){
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);

    for(int i=0;i<numStrips;i++){
        strips[i].numLEDs = 30; // default length, can set via FTC
        strips[i].leds = new CRGB[strips[i].numLEDs];
        strips[i].brightness = 255;
        strips[i].lastPattern = 0xFF;
        strips[i].lastUpdate = millis();
        strips[i].scannerPos = 0;
        strips[i].blinkState = false;
        FastLED.addLeds<WS2812, ledPins[i], GRB>(strips[i].leds, strips[i].numLEDs);
    }
    FastLED.clear(); FastLED.show();
}

void receiveEvent(int bytes){
    if(bytes<1) return;
    byte index = Wire.read();
    if(index>=numStrips) return;
    if(Wire.available()<6) return;

    byte high = Wire.read();
    byte low = Wire.read();
    int numLEDs = ((int)high << 8) | (int)low;
    byte r = Wire.read();
    byte g = Wire.read();
    byte b = Wire.read();
    byte brightness = Wire.read();
    byte pattern = Wire.read();

    LEDChannel &strip = strips[index];

    if(strip.numLEDs != numLEDs){
        delete[] strip.leds;
        strip.numLEDs = numLEDs;
        strip.leds = new CRGB[numLEDs];
        FastLED.addLeds<WS2812, ledPins[index], GRB>(strip.leds,numLEDs);
    }

    // Only update if changed
    if(r!=strip.lastR || g!=strip.lastG || b!=strip.lastB ||
       brightness!=strip.lastBrightness || pattern!=strip.lastPattern){
        strip.r = r; strip.g = g; strip.b = b;
        strip.brightness = brightness; strip.pattern = pattern;
        strip.lastUpdate = millis();
    }
    strip.lastR = r; strip.lastG = g; strip.lastB = b;
    strip.lastBrightness = brightness; strip.lastPattern = pattern;
}

void applySolid(LEDChannel &strip){
    for(int i=0;i<strip.numLEDs;i++)
        strip.leds[i] = CRGB(scale8(strip.r,strip.brightness),
                             scale8(strip.g,strip.brightness),
                             scale8(strip.b,strip.brightness));
}

void loop(){
    unsigned long now = millis();
    for(int i=0;i<numStrips;i++){
        LEDChannel &strip = strips[i];
        switch(strip.pattern){
            case SOLID: applySolid(strip); break;
            case RAINBOW:
                for(int j=0;j<strip.numLEDs;j++)
                    strip.leds[j] = CHSV((j*8 + now/10)%255, 255, strip.brightness);
                break;
            case BLINK:
                if(now - strip.lastUpdate >= 500){ strip.blinkState = !strip.blinkState; strip.lastUpdate = now; }
                for(int j=0;j<strip.numLEDs;j++)
                    strip.leds[j] = strip.blinkState ? CRGB(strip.r,strip.g,strip.b) : CRGB::Black;
                break;
            case SCANNER:
                if(now - strip.lastUpdate >= 50){ strip.scannerPos = (strip.scannerPos+1)%strip.numLEDs; strip.lastUpdate = now; }
                for(int j=0;j<strip.numLEDs;j++)
                    strip.leds[j] = (j==strip.scannerPos)?CRGB(strip.r,strip.g,strip.b):CRGB::Black;
                break;
            case BREATHE:
            {
                float f = (sin(now/500.0*3.14159)+1)/2;
                for(int j=0;j<strip.numLEDs;j++)
                    strip.leds[j] = CRGB(scale8(strip.r,(int)(strip.brightness*f)),
                                         scale8(strip.g,(int)(strip.brightness*f)),
                                         scale8(strip.b,(int)(strip.brightness*f)));
            }
                break;
        }
    }
    FastLED.show();
    delay(10);
}
