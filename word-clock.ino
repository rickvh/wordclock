//#define FASTLED_ESP32_RAW_PIN_ORDER
#define FASTLED_ALLOW_INTERRUPTS 1
#include "FastLED.h"
#include "FastLED_RGBW.h"
#include <FS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <TimeLib.h>
#include <HTTPClient.h>
#include <vector>

// Use time or LDR light sensor for auto brightness adjustment.
#define TIME_BRIGHTNESS
//#define LDR_BRIGHTNESS


struct Pixelreeks {
  CRGBW color;
  std::vector<int> pixels;

  Pixelreeks(CRGBW color, std::vector<int> pixels) {
    this->color = color;
    this->pixels = pixels;  
  }
};

typedef std::vector<Pixelreeks> t_image;

#define KERSTBOOM 0
#define SNEEUWPOP 1

t_image images[] = {
  // Kerstboom 
  {
    Pixelreeks(CRGBW(255, 255, 0, 0), {39}),
    Pixelreeks(CRGBW(0, 255, 0, 0), {5,28,49,20,94,60,83,109,45,9,52,16,53,17,59,97,87,98,86,61,66,103,79,67,78,68,108,77}),
    Pixelreeks(CRGBW(255, 0, 0, 0), {71})
  },

  // Sneeuwpop
  {
    Pixelreeks(CRGBW(0, 0, 255, 0), {39,42,5,32}),
    Pixelreeks(CRGBW(0, 0, 0, 200), {45,28,9,13,49,24,90,94,56,97,87,60,98,86,101,64,83,100,63,79,67,109,78,68,104,71,75}),
    Pixelreeks(CRGBW(255, 0, 0, 0), {95,52,16,20,53})
  }
};


const char* OTApass     = "test";

#define R_VALUE         0 //0
#define G_VALUE         255 //160
#define B_VALUE         0 //160
#define W_VALUE         0

#define R_VALUE_2         255 //200
#define G_VALUE_2         0 //200
#define B_VALUE_2         0 //0
#define W_VALUE_2         0

#define MIN_BRIGHTNESS  15
#define MAX_BRIGHTNESS  140
#define BRIGHTNESS      255 // legacy, keep at 255
int     lastBrightness  =  MIN_BRIGHTNESS;
int     dayHour         = 8; // Start increasing brightness
int     nightHour       = 17; // Start decreasing brightness

#define SYNC_INTERVAL   1200

#define NUM_LEDS        110
#define DATA_PIN        13 // D2 Pin on Wemos mini

#define LDR_PIN         A0
#define LDR_DARK        10
#define LDR_LIGHT       200

const int   timeZone        = 1;     // Central European Time
bool        autoDST         = true;

IPAddress   timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName   = "nl.pool.ntp.org";
const int   NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte        packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiUDP     Udp;
unsigned int localPort = 8888;

time_t getNtpTime();
void sendNTPpacket(IPAddress &address);

unsigned long   lastdisplayupdate   = 0;

// FastLED
//CRGB leds[NUM_LEDS];

// FastLED with RGBW
CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];

uint8_t targetlevels[NUM_LEDS];
uint8_t currentlevels[NUM_LEDS];

void rainbow();
bool isDST(int d, int m, int y);
bool isDSTSwitchDay(int d, int m, int y);
void updateBrightness();
int timeBrightness();

void setup() {
    pinMode(LDR_PIN, INPUT);
    
    Serial.begin(115200);

    WiFiManager wifiManager;


    uint32_t chipId = 0;
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    String ssid = "WordClock-" + String(chipId);
    wifiManager.autoConnect(ssid.c_str());

    Serial.println("Connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Starting UDP");
    Udp.begin(localPort);

    ArduinoOTA.setPassword(OTApass);

    ArduinoOTA.onStart([]() {
            Serial.println("Start");
            for(int i=0;i<NUM_LEDS;i++) {
                targetlevels[i] = 0;
                currentlevels[i] = 0;
                leds[i] = CRGB::Black;
            }
            leds[0] = CRGB::Red;
        
            FastLED.show();
            });
    ArduinoOTA.onEnd([]() {
            Serial.println("\nEnd");
            });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            int led = (progress / (total / NUM_LEDS));
            for(int i=0;i<led;i++) {
                targetlevels[i] = MAX_BRIGHTNESS;
                currentlevels[i] = MAX_BRIGHTNESS;
                leds[i] = CRGB::Red;
            }
            FastLED.show();
            });
    ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
            ESP.restart();
            });
    ArduinoOTA.begin();

    
    LEDS.addLeds<WS2812B,DATA_PIN>(ledsRGB, getRGBWsize(NUM_LEDS));
//    LEDS.setBrightness(87);
//    rainbow();
    LEDS.setBrightness(MIN_BRIGHTNESS);
    for(int i=0;i<NUM_LEDS;i++) {
        targetlevels[i] = 0;
        currentlevels[i] = 0;
        leds[i] = CRGB::Black;
    }

    FastLED.show();

    setSyncProvider(getNtpTime);
    setSyncInterval(SYNC_INTERVAL);
}

void rainbow() {
    uint8_t gHue = 0;
    while (gHue < 255) {
        EVERY_N_MILLISECONDS(20) {gHue++;}
        fill_rainbow(ledsRGB, NUM_LEDS, gHue, 1);
        FastLED.delay(1000/30); // 30FPS
    }
}

/*

   HET IS X UUR
   HET IS VIJF OVER X
   HET IS TIEN OVER X
   HET IS KWART OVER X
   HET IS TIEN VOOR HALF (X+1)
   HET IS VIJF VOOR HALF (X+1)
   HET IS HALF (X+1)
   HET IS VIJF OVER HALF (X+1)
   HET IS TIEN OVER HALF (X+1)
   HET IS KWART VOOR (X+1)
   HET IS TIEN VOOR (X+1)
   HET IS VIJF VOOR (X+1)
   HET IS (X+1) UUR
   ...
*/

#define HETIS 0
#define VIJF  13
#define TIEN  14
#define KWART 15
#define VOOR1  16
#define OVER1  17
#define HALF  18
#define UUR   19
#define VOOR2 20
#define OVER2 21

#define PIEK 0
#define BOOM 1
#define STAM 2

int selectedLed;

std::vector<std::vector<int>> ledsbyword = {
    {0,37,38,36,39}, // HET IS
    {17,19,54},            // een
    {58,89,95,57},      // twee
    {91,93,55,92},      // drie
    {96,88,59,97},      // vier
    {87,60,98,86},         // vijf
    {61,99,85},         // zes
    {102,65,82,101,64},   // zeven
    {80,66,103,79},      // acht
    {100,63,84,107,62},   // negen
    {67,109,78,68},      // tien
    {108,77,69},         // elf
    {73,81,72,74,104,71},// twaalf
    {35,40,3,34},    // VIJF
    {30,43,6,31},    // TIEN
    {9,46,27,10,47}, // KWART
    {41,4,33,26},    // VOOR1
    {7,44,29,8},    // OVER1
    {22,14,50,23},    // HALF
    {70,76,106},      // UUR
    {51,15,21,52},          // VOOR2
    {12,48,25,11}          // OVER2
};


void handleSerialInput() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    int getal = Serial.parseInt();
    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      // constrain the values to 0 - 255 and invert
      // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"
      if (getal < 0 || getal >= NUM_LEDS) {
        Serial.println("Voer getal in tussen 0 en 109");
        break;
      }
      
      selectedLed = getal;
    }
  }
}

void loop() {


    handleSerialInput();

  
    // put your main code here, to run repeatedly:
    ArduinoOTA.handle();
    //  Serial.println("loop");
//    Serial.print(analogRead(LDR_PIN));
//    Serial.print(", ");
//    Serial.print(lastBrightness);
//    Serial.print("   ");
//    Serial.print(hour());
//    Serial.print(":");
//    Serial.println(minute());

    // only update clock every 50ms
    if(millis()-lastdisplayupdate > 50) {
        lastdisplayupdate = millis();
    }else{
        return;
    }

    #ifdef TIME_BRIGHTNESS
      LEDS.setBrightness(timeBrightness());
    #endif
    #ifdef LDR_BRIGHTNESS
      updateBrightness();
    #endif

    // if not connected, then show waiting animation
    if(timeStatus() == timeNotSet) {
        // show initialisation animation
        Serial.println("time not yet known");
        for(int i=0;i<NUM_LEDS;++i){ //blank rest
            leds[i] = CRGB::Black;
        }

        float phase = ((float)(millis()%2000)) / 1000;
        if(phase > 1) phase = 2.0f-phase;
        for(int i=0;i<4;++i){  // the scanner moves from 0 to(inc) 5, but only 1..4 are actually shown on the four leds
            float intensity = abs((float)(i-1)/3-phase);
            intensity = sqrt(intensity);
            leds[i] = CRGB(255-(255*intensity),0,0);
        }
        FastLED.show();
        return;
    }

    
    time_t t = now();

    int imageCount = second() / 15;

    if (hour() >= 22 || hour() <= 7)
      imageCount = 0;

    if (imageCount == 1) {
      showImage(0);
    } else if (imageCount == 3) {
      showImage(1);
    } else {
      showKlok();
    }
    
    
    // Update LEDs
    FastLED.show();
}

void showKlok() {
      // calculate target brightnesses:
    int current_hourword = hour();
    if(current_hourword>12) current_hourword = current_hourword - 12; // 12 hour clock, where 12 stays 12 and 13 becomes one
    if(current_hourword==0) current_hourword = 12;            // 0 is also called 12

    int next_hourword = hour()+1;
    if(next_hourword>12) next_hourword = next_hourword - 12;      // 12 hour clock, where 12 stays 12 and 13 becomes one
    if(next_hourword==0) next_hourword = 12;              // 0 is also called 12


    int speed = 2;
  
  
    // move current brightness towards target brightness:
    for(int i=0;i<NUM_LEDS;++i) {
        if(currentlevels[i] < targetlevels[i]) {
            currentlevels[i] = std::min(BRIGHTNESS,currentlevels[i]+speed);
        }
        if(currentlevels[i] > targetlevels[i]) {
            currentlevels[i] = std::max(0,currentlevels[i]-speed);
        }

        // output the value to led: according to the function x^2/255 to compensate for the perceived brightness of leds which is not linear
        leds[i] = CRGBW(
                currentlevels[i]*currentlevels[i]*R_VALUE/65025,
                currentlevels[i]*currentlevels[i]*G_VALUE/65025,
                currentlevels[i]*currentlevels[i]*B_VALUE/65025,
                currentlevels[i]*currentlevels[i]*W_VALUE/65025);
    }


    for(int i=0;i<NUM_LEDS;i++) {
        targetlevels[i] = 0;
    }


    // RVH: Custom debug optie
    //targetlevels[selectedLed] =  255;

    for(int l : ledsbyword[HETIS]) { targetlevels[l] = 255; }
    switch((minute()%60)/5) {
        case 0:
            for(int l : ledsbyword[current_hourword])   { targetlevels[l] = 255; }
            for(int l : ledsbyword[UUR])        { targetlevels[l] = 255; }
            break;
        case 1:
            for(int l : ledsbyword[VIJF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[OVER1])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[current_hourword]) { targetlevels[l] = 255; }
            break;
        case 2:
            for(int l : ledsbyword[TIEN])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[OVER2])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[current_hourword])   { targetlevels[l] = 255; }
            break;
        case 3:
            for(int l : ledsbyword[KWART])        { targetlevels[l] = 255; }
            for(int l : ledsbyword[OVER2])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[current_hourword])   { targetlevels[l] = 255; }
            break;
        case 4:
            for(int l : ledsbyword[TIEN])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[VOOR1])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[HALF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 5:
            for(int l : ledsbyword[VIJF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[VOOR1])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[HALF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 6:
            for(int l : ledsbyword[HALF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 7:
            for(int l : ledsbyword[VIJF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[OVER1])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[HALF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 8:
            for(int l : ledsbyword[TIEN])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[OVER1])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[HALF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 9:
            for(int l : ledsbyword[KWART])        { targetlevels[l] = 255; }
            for(int l : ledsbyword[VOOR2])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 10:
            for(int l : ledsbyword[TIEN])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[VOOR1])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
        case 11:
            for(int l : ledsbyword[VIJF])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[VOOR2])         { targetlevels[l] = 255; }
            for(int l : ledsbyword[next_hourword])    { targetlevels[l] = 255; }
            break;
    }


    showKlokUpdateKleur();
    
    // the minute leds at the bottom:
    //for(int i=4-(minute()%5);i<4;++i) {
    //    targetlevels[i] = 255;
    //}  
}

void showKlokUpdateKleur() {
  // FIX 2e kleur
    for(int uur = 1; uur <=12; uur++) {
        for(int l : ledsbyword[uur]) {
            if (currentlevels[l] > 0) {
                leds[l] = CRGBW(
                  currentlevels[l]*currentlevels[l]*R_VALUE_2/65025,
                  currentlevels[l]*currentlevels[l]*G_VALUE_2/65025,
                  currentlevels[l]*currentlevels[l]*B_VALUE_2/65025,
                  currentlevels[l]*currentlevels[l]*W_VALUE_2/65025);
            }
        }
    }

    // FIX 'het is'
     for(int l : ledsbyword[0]) {
        if (currentlevels[l] > 0) {
            leds[l] = CRGBW(
              currentlevels[l]*currentlevels[l]*0/65025,
              currentlevels[l]*currentlevels[l]*0/65025,
              currentlevels[l]*currentlevels[l]*0/65025,
              currentlevels[l]*currentlevels[l]*200/65025);
        }
    }
}

void showImage(int index) {
    for(int i=0;i<NUM_LEDS;++i){ //blank rest
      targetlevels[i] = 0;
      leds[i] = CRGB::Black;
    }

    for (auto & reeks : images[index]) {
      for (int l : reeks.pixels) {
        leds[l] = reeks.color;
      }
    }
}

/*-------- NTP code ----------*/

time_t getNtpTime()
{
    IPAddress ntpServerIP; // NTP server's ip address

    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    Serial.println("Transmit NTP Request");
    // get a random server from the pool
    WiFi.hostByName(ntpServerName, ntpServerIP);
    Serial.print(ntpServerName);
    Serial.print(": ");
    Serial.println(ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
        int size = Udp.parsePacket();
        if (size >= NTP_PACKET_SIZE) {
            Serial.println("Receive NTP Response");
            Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            // New time in seconds since Jan 1, 1970
            unsigned long newTime = secsSince1900 - 2208988800UL +
                timeZone * SECS_PER_HOUR;

            // Auto DST
            if (autoDST) {
                if (isDSTSwitchDay(day(newTime), month(newTime), year(newTime))) {
                    if (month(newTime) == 3 && hour(newTime) >= 2) {
                        newTime += SECS_PER_HOUR;
                    } else if (month(newTime) == 10 && hour(newTime) < 2) {
                        newTime += SECS_PER_HOUR;
                    }
                } else if (isDST(day(newTime), month(newTime), year(newTime))) {
                    newTime += SECS_PER_HOUR;
                }
            }

            setSyncInterval(SYNC_INTERVAL);
            return newTime;
        }
    }
    Serial.println("No NTP Response :-(");
    // Retry soon
    setSyncInterval(10);
    return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}

// Check if Daylight saving time (DST) applies
// Northern Hemisphere - +1 hour between March and October
bool isDST(int d, int m, int y){
    bool dst = false;
    dst = (m > 3 && m < 10); // October-March

    if (m == 3){
        // Last sunday of March
        dst = (d >= ((31 - (5 * y /4 + 4) % 7)));
    }else if (m == 10){
        // Last sunday of October
        dst = (d < ((31 - (5 * y /4 + 1) % 7)));
    }

    return dst;
}

bool isDSTSwitchDay(int d, int m, int y){
    bool dst = false;
    if (m == 3){
        // Last sunday of March
        dst = (d == ((31 - (5 * y /4 + 4) % 7)));
    }else if (m == 10){
        // Last sunday of October
        dst = (d == ((31 - (5 * y /4 + 1) % 7)));
    }
    return dst;
}

int readAvgAnalog(int pin, byte numReadings, int readingDelay){
    int readingsTotal = 0;

    for (int i = 0; i < numReadings; i++) {
        readingsTotal += analogRead(pin);
        delay(readingDelay);
    }

    return readingsTotal / numReadings;
}

void updateBrightness(){
    int brightness = map(readAvgAnalog(LDR_PIN,50,2), LDR_DARK, LDR_LIGHT, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    brightness = constrain(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);

    // Smooth brightness change
    int difference = abs(brightness - lastBrightness);
    if (brightness > lastBrightness) {
        brightness = lastBrightness + (1 + 30 * difference / 189 );
    }else if (brightness < lastBrightness) {
        brightness = lastBrightness - (1 + 30 * difference / 189 );
    }

    lastBrightness = brightness;

    LEDS.setBrightness(brightness);
}

int timeBrightness() {
    if (hour() > dayHour && hour() < nightHour) {
        return MAX_BRIGHTNESS;
    } else if (hour() < dayHour || hour() > nightHour) {
        return MIN_BRIGHTNESS;
    } else if (hour() == dayHour) {
        return constrain(
                map(minute(), 0, 29, MIN_BRIGHTNESS, MAX_BRIGHTNESS),
                MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    } else if (hour() == nightHour) {
        return constrain(
                map(minute(), 0, 29, MAX_BRIGHTNESS, MIN_BRIGHTNESS),
                MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    }
}
