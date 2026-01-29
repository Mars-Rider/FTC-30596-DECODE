#include <Wire.h>
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ---------------- WebSocket ----------------
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
bool WiFi = false;
bool started = false;
bool clientConnected = false;

// ---------------- CONFIG ----------------
#define MAX_CHANNELS 5
#define LEDS_PER_CHANNEL 300 // default, can be updated from packet
#define I2C_ADDR 0x08

// ---------------- DATA STRUCTURES ----------------
struct LEDChannel {
    uint8_t pin = 0;

    unsigned int numLEDs = LEDS_PER_CHANNEL;

    //Colors
    CHSV* leds = new CHSV[LEDS_PER_CHANNEL];
    int h = 0;
    uint8_t s = 255;
    int brightness = 0;//Doesn't show until new data

    //Color Parameter
    bool rainbow = false;
    int colorPeriod = 10000;
    int hueMin = 0;
    int hueMax = 0;

    //Pattern
    uint8_t pattern = 1;

    //Pattern Parameter
    int trailLength = 4;
    int sourceCenter = numLEDs / 2;
    bool gradient = false;
    bool moving = false;
    int period = 500;
    bool direction = true;

    unsigned long lastUpdate = millis();
};

// ---------------- CHANNELS ----------------
LEDChannel channels[MAX_CHANNELS];
Socket socket[MAX_CHANNELS];
int numChannels = 0;

// ---------------- FUNCTION DECLARATIONS ----------------
void applyPattern(LEDChannel &ch);
void updateLEDs();
void i2cReceiveEvent(int bytesReceived);

// ---------------- SETUP ----------------
void setup() {
    Wire.begin(I2C_ADDR);
    Wire.onReceive(i2cReceiveEvent);

    channels[0].pin = 0;//Replace with the actual
    channels[1].pin = 1;
    channels[2].pin = 2;
    channels[3].pin = 3;
    channels[4].pin = 4;
}

unsigned long lastMillis = 0;
double fps;

// ---------------- LOOP ----------------
void loop() {
    handleWebSocket();

    updateLEDs();
    FastLED.clear();
    FastLED.show();

    unsigned long currMillis = millis();
    fps = 1000/(currMillis - lastMillis);
    lastMillis = currMillis;
    Serial.print("FPS: ");
    Serial.println(fps);
}

// ---------------- UPDATE PATTERNS & COLORS ----------------
void updateLEDs() {
    unsigned long now = millis();
    if(!WiFi){
        for (int ch = 0; ch < numChannels; ch++) {
            LEDChannel &c = channels[ch];
            if (now - c.lastUpdate >= c.period) {
                applyPattern(c);
                c.lastUpdate = now;
            }
        }
    }
}

// ---------------- ALL PATTERNS ----------------
void applyPattern(LEDChannel &ch) {
    switch (ch.pattern) {
        case 1: // BLINK
            for (int i = 0; i < ch.numLEDs; i++)
                ch.leds[i] = (ch.position % 2 == 0) ? CHSV(ch.h, ch.s, ch.brightness)  : CHSV::Black;
            ch.position++;
            break;

        case 2: // LARSON / SCANNER
                // Fade all LEDs to create a trailing effect
            for (int i = 0; i < ch.numLEDs; i++)
            {
                ch.leds[i].fadeToBlackBy(ch.trailLength); // adjust for longer/shorter trail
            }

            // Light the current position
            ch.leds[pos] = ch.leds[pos];

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

    if (bytesRecieved > 32) bytesRecieved = 32;

    int bytesRead = 0;
    uint8_t data[bytesRecieved]
    while (Wire.available() && bytesRead < bytesRecieved) {
        buffer[bytesRead++] = Wire.read();
    }

    if(bytesRecieved <= 2){
        handleStartPacket(data);
    }
    /*IN DEVELOPMENT
    else if (bytesRecieved <= 10){
        handleLEDPacket(data);
    }*/
    else if (bytesRecieved <= 25){
        handleStartPacket(data);
    }
    else if (bytesRecieved <= 32){
        handleWiFiPacket(data);
    }
}

void handleStartPacket(uint8_t* data){//Starting packet
    int chIndex = data[0];
    if (chIndex >= numChannels)
        numChannels = chIndex + 1;

    LEDChannel &ch = channels[chIndex];

    int stripID = data[1];

    ch.numLEDs = ((int)data[2] << 8) | data[3];
    if (ch.numLEDs > LEDS_PER_CHANNEL)
        ch.numLEDs = LEDS_PER_CHANNEL;
    ch.leds = new CHSV[ch.numLEDs];

    if(!WiFi){ 
        switch (stripID)
        {
        case 1:
            FastLED.addLeds<WS2812, channels[chIndex].pin, GRB>(channels[chIndex].leds, channels[chIndex].numLEDs);
            break;
        case 2:
            FastLED.addLeds<SK6812, channels[chIndex].pin, GRB>(channels[chIndex].leds, channels[chIndex].numLEDs);
            break;
        }   
    } 
}

/*IN DEVELOPMENT
void handleLEDPacket(uint8_t* data){}//Singular LED Changes*/

void handleStripPacket(uint8_t* data){//Full strip packet
    int chIndex = data[0];
    if (chIndex < 0 || chIndex >= numChannels) return;

    LEDChannel &ch = channels[chIndex];

    //Color
    ch.h = ((int)data[1] << 8) | data[2];
    ch.s = data[3];
    ch.brightness = data[4];

    // Set all LEDs to the base color (SOLID fallback)
    for (int i = 0; i < ch.numLEDs; i++) {
        ch.leds[i] = CHSV(constrain(socket.h, 0, 360, 0, 255), s, 255);
    }

    //Color Parameters
    ch.rainbow = data[5] != 0;
    ch.colorPeriod = ((int)data[6] << 8) | data[7];
    ch.hueMin = ((int)data[8] << 8) | data[9];
    ch.hueMax = ((int)data[10] << 8) | data[11];

    // Pattern
    ch.pattern = data[12];

    // Pattern parameters
    ch.trailLength = data[13];
    ch.sourceCenter = ((int)data[14] << 8) | data[15];
    ch.sourceLength = data[16]
    ch.gradient = data[17] != 0;
    ch.moving = data[18] != 0;
    ch.period = ((int)data[19] << 8) | data[20];
    ch.direction = data[21] != 0;

    // Make sure position state is valid
    if (ch.position >= ch.numLEDs) ch.position = round(ch.numLEDS/2);
}

void handleWiFiPacket(uint8_t *data)
{
    if(WiFi != true){
        WiFi = true;
        //Clear all leds for space
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            if (channels[i].leds != nullptr)
            {
                FastLED.removeLedsFromController(channels[i].leds); // remove from FastLED
                delete[] channels[i].leds;                          // free memory
                channels[i].leds = nullptr;                         // safety
            }
        }
        FastLED.clear();
        FastLED.show();

        char rxBuffer[33]; // 32 + 1 for null terminator

        memcpy(rxBuffer, data, 32);
        rxBuffer[32] = '\0'; // null terminate
        trimSSID(rxBuffer);  // remove trailing spaces

        startWebSocket(rxBuffer);
    }
}

void trimSSID(char *str)
{
    int len = strlen(str);
    while (len > 0 && str[len - 1] == ' ')
    {
        str[len - 1] = '\0';
        len--;
    }
}

void startWebSocket(const char *ssid){
    // Start Websocket
    if (!LittleFS.begin())
    {
        SERIAL.println("An error has occurred while mounting LittleFS");
    }
    else
    {
        SERIAL.println("LittleFS mounted successfully");
    }

    WiFi.softAP(ssid);

    IPAddress myIP = WiFi.softAPIP();
    SERIAL.print("AP IP address: ");
    SERIAL.println(myIP);

    server.onNotFound([]() { // Check for file to upload the html file
        if (!handleFileRead(server.uri()))
            server.send(404, "text/plain", "FileNotFound");
    });

    server.begin();
    SERIAL.println("HTTP server started");

    // Setup Websocket for PID and Gyro Tuning/Testing
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    SERIAL.println("WebSocket server started.");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        SERIAL.printf("[%u] Disconnected!\n", num);
        clientConnected = false;
        break;
    case WStype_CONNECTED:
        {
            IPAddress ip = webSocket.remoteIP(num);
            SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            // webSocket.sendTXT(num, "Connected");  //sends to the clioent thst doed the event
            clientConnected = true;
            
            //Send starting data

            DynamicJsonDocument doc(1024);

            JsonArray array = doc.to<JsonArray>();

            for (int i = 0; i < numChannels; i++)
            {
                JsonObject obj = array.createNestedObject();
                obj["numLEDs"] = channels[i].numLEDs;
                obj["h"] = channels[i].h;
                obj["s"] = channels[i].s;
                obj["brightness"] = channels[i].brightness;
                obj["rainbow"] = channels[i].rainbow;
                obj["colorPeriod"] = channels[i].colorPeriod;
                obj["hueMin"] = channels[i].hueMin;
                obj["hueMax"] = channels[i].hueMax;
                obj["pattern"] = channels[i].pattern;
                obj["trailLength"] = channels[i].trailLength;
                obj["sourceCenter"] = channels[i].sourceCenter;
                obj["gradient"] = channels[i].gradient;
                obj["moving"] = channels[i].moving;
                obj["period"] = channels[i].period;
                obj["direction"] = channels[i].direction;
            }

            String data;
            serializeJson(doc, data); // Convert JSON document to string

            webSocket.broadcastTXT(data); // Send back current gyro and acc data to ALL clients
            break;
        }
    }
}

// update websocket and send data
void handleWebSocket(){
    if (!WiFi || numChannels == 0) return;

    // Get Updates
    server.handleClient();
    webSocket.loop();

    DynamicJsonDocument doc(1024);

    JsonArray array = doc.to<JsonArray>();

    for (int i = 0; i < numChannels; i++)
    {
        JsonObject obj = array.createNestedObject();
        obj["numLEDs"] = channels[i].numLEDs;
        obj["h"] = channels[i].h;
        obj["s"] = channels[i].s;
        obj["brightness"] = channels[i].brightness;
        obj["rainbow"] = channels[i].rainbow;
        obj["colorPeriod"] = channels[i].colorPeriod;
        obj["hueMin"] = channels[i].hueMin;
        obj["hueMax"] = channels[i].hueMax;
        obj["pattern"] = channels[i].pattern;
        obj["trailLength"] = channels[i].trailLength;
        obj["sourceCenter"] = channels[i].sourceCenter;
        obj["gradient"] = channels[i].gradient;
        obj["moving"] = channels[i].moving;
        obj["period"] = channels[i].period;
        obj["direction"] = channels[i].direction;
    }

    String data;
    serializeJson(doc, data); // Convert JSON document to string

    webSocket.broadcastTXT(data); // Send back current gyro and acc data to ALL clients
}

bool handleFileRead(String path)
{
    if (!clientConnected)
    {
        SERIAL.println("handleFileRead: " + path);
        if (path.endsWith("/"))
        {
            path += "index.html";
        }

        SERIAL.print("Path: ");
        SERIAL.println(path);

        if (LittleFS.exists(path))
        {
            File file = LittleFS.open(path, "r");
            size_t sent = server.streamFile(file, "text/html");
            file.close();
            return true;
        }
        else
        {
            SERIAL.println("Path doesn't exist");
        }
    }
    return false;
}
