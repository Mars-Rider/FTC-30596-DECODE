#include <Wire.h>
#include <FastLED.h>

// ---------------- CONFIG ----------------
#define MAX_CHANNELS 5
#define LEDS_PER_CHANNEL 60 // default, can be updated from packet
#define DATA_PIN 5          // WS2812B data pin
#define I2C_ADDR 0x08

// ---------------- DATA STRUCTURES ----------------
struct LEDChannel {
    int pin = 0;

    int numLEDs = LEDS_PER_CHANNEL;
    int brightness = 255;
    int pattern = 3; // SOLID default
    int trailLength = 4;
    int radarCenter = numLEDs / 2;
    // radarTrail = 3;
    int twinkleHueMin = 0;
    int twinkleHueMax = 255;
    bool gradientMoving = false;
    int period = 500;
    int position = 0;
    bool direction = true;
    unsigned long lastUpdate = millis();
    CHSV* leds = new CHSV[LEDS_PER_CHANNEL];
};

// ---------------- CHANNELS ----------------
LEDChannel channels[MAX_CHANNELS];
int numChannels = 0;

// ---------------- FUNCTION DECLARATIONS ----------------
void applyPattern(LEDChannel &ch);
void updateLEDs();
void i2cReceiveEvent(int bytesReceived);

// ---------------- SETUP ----------------
void setup() {
    Wire.begin(I2C_ADDR);
    Wire.onReceive(i2cReceiveEvent);

    channels[0].pin = 0;
    channels[1].pin = 1;
    channels[2].pin = 2;
    channels[3].pin = 3;
    channels[4].pin = 4;

    // Initialize channels (up to MAX_CHANNELS)
    for (int i = 0; i < MAX_CHANNELS; i++) {
        FastLED.addLeds<WS2812, channels[i].pin, GRB>(channels[i].leds, channels[i].numLEDs);
    }
}

// ---------------- LOOP ----------------
void loop() {
    updateLEDs();
    FastLED.show();
}

// ---------------- UPDATE PATTERNS ----------------
void updateLEDs() {
    unsigned long now = millis();
    for (int ch = 0; ch < numChannels; ch++) {
        LEDChannel &c = channels[ch];
        if (now - c.lastUpdate >= c.period) {
            applyPattern(c);
            c.lastUpdate = now;
        }

        for (int i = 0; i < c.numLEDs; i++)
        {
            c[i].nscale8_video((c.brightness/100)*255); // 128 is ~50% brightness
        }
    }
}

// ---------------- APPLY PATTERN ----------------
void applyPattern(LEDChannel &ch) {
    switch (ch.pattern) {
        case 1: // BLINK
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = (ch.position % 2 == 0) ? ch.leds[i]  : CHSV::Black;
            ch.position++;
            break;

        case 2: // LARSON / SCANNER
                // Fade all LEDs to create a trailing effect
            for (int i = 0; i < ch.numLEDs; i++)
            {
                leds[i].fadeToBlackBy(ch.trailLength); // adjust for longer/shorter trail
            }

            // Light the current position
            ch.leds[pos] = ch.leds[pos];

            FastLED.show();
            delay(30); // adjust speed

            // Move to next LED
            pos++;

            // Reset to start when reaching the end
            if (pos >= NUM_LEDS)
            {
                pos = 0;
            }
            break;

        case 3: // SOLID
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CHSV(ch.twinkleHueMin, 255, ch.brightness);
            break;

        case 4: // BREATHE
        {
            int val = (sin(millis() * 2.0 * PI / ch.period) + 1.0) * 0.5 * ch.brightness;
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CHSV(ch.twinkleHueMin, 255, val);
        }
            break;

        case 5: // GRADIENT_STATIC
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CHSV(map(i, 0, ch.numLEDs - 1, ch.twinkleHueMin, ch.twinkleHueMax), 255, ch.brightness);
            break;

        case 6: // GRADIENT_MOVING
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CHSV(map((i + ch.position) % ch.numLEDs, 0, ch.numLEDs - 1, ch.twinkleHueMin, ch.twinkleHueMax), 255, ch.brightness);
            ch.position = (ch.position + 1) % ch.numLEDs;
            break;

        case 7: // PINGPONG
            // First, fade all LEDs to create the trailing effect
            for (int i = 0; i < NUM_LEDS; i++)
            {
                leds[i].fadeToBlackBy(50); // Adjust 50 for faster/slower fading
            }

            // Light the current position
            leds[pos] = COLOR;

            // Move the scanner
            pos += direction;

            // Reverse at the ends
            if (pos <= 0 || pos >= NUM_LEDS - 1)
            {
                direction = -direction;
            }

            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = (abs(i - ch.position) < ch.trailLength) ? CHSV(ch.twinkleHueMin, 255, ch.brightness) : CRGB::Black;
            ch.position += ch.trailLength;
            if (ch.position >= ch.numLEDs) ch.position = 0;
            break;

        case 8: // FLICKER
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CHSV(random(ch.twinkleHueMin, ch.twinkleHueMax), 255, random(ch.brightness / 2, ch.brightness));
            break;

        case 9: // HEARTBEAT
        {
            float t = sin(millis() * 2.0 * PI / ch.period);
            int val = (t * t) * ch.brightness; // squared sine
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CHSV(ch.twinkleHueMin, 255, val);
        }
            break;

        case 10: // RADAR
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = (i >= ch.radarCenter - ch.trailLength && i <= ch.radarCenter + ch.trailLength) ? CHSV(ch.twinkleHueMin, 255, ch.brightness) : CRGB::Black;
            ch.radarCenter = (ch.radarCenter + 1) % ch.numLEDs;
            break;

        default:
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = CRGB::Black;
            break;
    }
}

void i2cReceiveEvent(int bytesReceived) {
    while (Wire.available() >= 17) { // each packet is 17 bytes
        uint8_t data[17];
        for (int i = 0; i < 17; i++) {
            data[i] = Wire.read();
        }

        int chIndex = data[0];
        if (chIndex < 0 || chIndex >= MAX_CHANNELS) continue;

        LEDChannel &ch = channels[chIndex];

        // Number of LEDs
        ch.numLEDs = ((int)data[1] << 8) | data[2];
        if (ch.numLEDs > LEDS_PER_CHANNEL) ch.numLEDs = LEDS_PER_CHANNEL;

        // Base color (already computed if VALUE)
        uint8_t h = data[3];
        uint8_t s = data[4];
        uint8_t v = data[5];

        // Set all LEDs to the base color (SOLID fallback)
        for (int i = 0; i < ch.numLEDs; i++) {
            ch.leds[i] = CHSV(h, s, v);
        }

        // Brightness
        ch.brightness = data[6];

        // Pattern
        ch.pattern = data[7];

        // Pattern parameters
        ch.trailLength = data[8];
        ch.radarCenter = ((int)data[9] << 8) | data[10];
        //ch.radarTrail = data[11];
        ch.twinkleHueMin = ((int)data[11] << 8) | data[12];
        ch.gradientMoving = data[13] != 0;
        ch.period = ((int)data[14] << 8) | data[15];
        ch.direction = data[16] != 0;

        // Make sure position state is valid
        if (ch.position >= ch.numLEDs) ch.position = 0;

        // Track number of channels
        if (chIndex >= numChannels) numChannels = chIndex + 1;
    }
}

