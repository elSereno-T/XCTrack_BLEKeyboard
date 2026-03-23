#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HijelHID_BLEKeyboard.h>
#include <Keypad.h>
#include <shared.h>

#include <Wire.h>
#include <SSD1306Wire.h>


// #ifdef __has_include
//   #if __has_include("wifi.h")
//     #include "wifi.h"
//   #endif
// #endif
// #ifndef WIFI_SSID
//     #define WIFI_SSID ""
// #endif
// #ifndef WIFI_PWD
//     #define WIFI_PWD ""
// #endif


uint16_t NextTryDelta = 10000;
#define TIMEOUT_MS 5000

struct {
    unsigned long now;
    unsigned long camera;
    unsigned long system;
    unsigned long phone;
    unsigned long gps;
    unsigned long display;
    unsigned long display_timeout;
} timestamps;

typedef enum {INITIAL_BOOT,WAIT_FOR_BUTTON_HOLD, WAIT_FOR_TIME, WAIT_FOR_BUTTON_RELEASE, WAIT_FOR_CONFIRMATION, SETUP, RUNNING, INITIAL_SHUTDOWN, WAIT_FOR_SHUTDOWN_CONFIRMATION, SHUTDOWN, OFF} SystemState;
const String SystemStateString[] = {"INITIAL BOOT", "WAIT FOR BUTTON HOLD","WAIT FOR TIME", "WAIT FOR BUTTON RELEASE", "WAIT FOR CONFIRMATION", "SETUP", "RUNNING", "INITIAL SHUTDOWN", "WAIT FOR SHUTDOWN CONFIRMATION", "SHUTDOWN", "OFF"};
const String ShortSystemStateString[] = {"HOLD", "HOLD","HOLD", "RELEASE", "CONFIRM", "SETUP", "RUNNING", "RELEASE", "CONFIRM", "SHUTDOWN", "OFF"};
SystemState sysState = OFF;

typedef enum {DISCHARGING, CHARGING} PhoneState;
const String PhoneStateString[] = {"DISCHARGING", "CHARGING"};
PhoneState PhoneCharging = CHARGING;
bool toggleCharging = false;
bool ChargerInitialized = false;

typedef enum {POWER_OFF, POWER_ON} GPSState;
const String GPSStateStateString[] = {"POWER_OFF", "POWER_ON"};
GPSState GPS_ON_OFF = POWER_OFF;


typedef enum  {DISPLAY_OFF, DISPLAY_SYS_STATE, DISPLAY_CAMERA, DISPLAY_SETUP} DisplayState;
const String DisplayStateString[] = {"OFF", "SYS_STATE", "CAMERA", "SETUP"};
DisplayState dispState = DISPLAY_OFF;
DisplayState prevDispState = DISPLAY_OFF;
bool updateDisplay = false;

#define DISPLAY_REFRESH_RATE 40
#define DISPLAY_BLINK_RATE 1000

bool anyKey = false;

int stateOfChargeMain=75;


#define Vol_p ((char) '+')
#define Vol_m ((char) '-')
#define ESC ((char) 'e')
#define Home ((char) 'h')
#define Power ((char) 'o')
#define Backspace ((char) 'b')
#define Enter ((char) 'n')
#define ND ((char) ' ')
#define Tab ((char) 's')
#define BL ((char) 'l')

#define XCT ((char) 'x')
#define REC ((char) 'a')
#define AltTab ((char) 's')

const String key_chars =  String(ESC) + String(Backspace) + String(Power) + String(Enter) + String(Tab) ;
const uint8_t key_array[] = {KEY_ESCAPE,   KEY_BACKSPACE,      KEY_POWER,      KEY_RETURN,     KEY_TAB};
const String media_chars =     String(Vol_p) +  String(Vol_m) +    String(Home) +      String(BL);
const uint16_t media_array[] = {MEDIA_VOLUME_UP, MEDIA_VOLUME_DOWN, MEDIA_BROWSER_HOME, MEDIA_DISPLAY_BACKLIGHT};
const String no_repeat = String(Power) + String(AltTab) + String(Enter)+ String(Home) + String(REC) + String(ESC) + String(XCT) + String(Backspace);

const byte ROWS = 4;
const byte COLS = 3;

const char KEYS[ROWS][COLS] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {Vol_m,'0',Vol_p}
};

const char ALT_KEYS[ROWS][COLS] = {
    {ESC,'2',Home},
    {'4',XCT,'6'},
    {REC,'8',AltTab},
    {Backspace,Power,Enter}
};

const char MENU_KEYS[ROWS][COLS] = {
    {ESC,'^',0},
    {'<',Enter,'>'},
    {0,'v',0},
    {0,0,Enter}
};


Keypad KeypadMain;
Keypad KeypadPower;
Keypad KeypadMenu;


const uint8_t row_GPIOs[ROWS] = {D10,D9,D8,D7};
const uint8_t col_GPIOs[COLS] = {D3,D1,D0};

const byte Power_row = 3;
const byte confirm_row = 1;
const byte Power_col = 1;

const uint8_t power_row_GPIOs[] = {row_GPIOs[Power_row],row_GPIOs[confirm_row] };
const uint8_t power_row_GPIO = row_GPIOs[Power_row];
const uint8_t confirm_row_GPIO = row_GPIOs[confirm_row];
const uint8_t power_col_GPIO = col_GPIOs[Power_col];

const uint16_t POWER_CYCLE_DELAY = 3000;

HijelHID_BLEKeyboard bleKeyboard("XCTrack Keypad", "TS", 50);
bool keyboardAvailable = false;


// unsigned long now;


bool recording = false;
unsigned long pre_shutdown_release = 0;

RTC_DATA_ATTR int bootCount = 0;



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);

bool displayReady=false;

bool blink=false;


const char* knownMacs[] = { "98:3d:ae:ab:e2:7a" , "98:3d:ae:ac:92:9a" };

const int   knownMacCount = sizeof(knownMacs) / sizeof(knownMacs[0]);


struct BleClient {
    NimBLEClient*               pClient     = nullptr;
    NimBLERemoteCharacteristic* pCmd        = nullptr;
    NimBLERemoteCharacteristic* pAck        = nullptr;
    NimBLERemoteCharacteristic* pBatt       = nullptr;
    uint8_t                     ackValue    = 0;
    bool                        ackReceived = false;
    bool                        waitingForAck = false;
    uint8_t                     battPercent = 255;   // 255 = unknown
    bool                        newbattvalue = false;
    bool                        connected   = false;
    bool                        recording   = false;
    unsigned long               nextTry     = 0;
    unsigned long               commandSent = 0;         
};

static BleClient clients[knownMacCount];
void sendCmd(uint8_t cmd) {
    Serial.printf("[HOST] Sent CMD: 0x%02X\n", cmd);
    for (int i = 0; i < knownMacCount; i++) {
        if (!clients[i].connected) continue;
        clients[i].ackReceived = false;
        clients[i].waitingForAck = true;
        clients[i].commandSent = timestamps.now;
        clients[i].pCmd->writeValue(&cmd, 1, false);
    }
}

bool waitForAck(byte ACK_MSG) {
    bool any = false;
    bool all = true;
    for (int i = 0; i < knownMacCount; i++) {
        if (!clients[i].waitingForAck) continue;
        if (clients[i].ackReceived && (clients[i].ackValue == ACK_MSG)){
            Serial.printf("[HOST] ACK received from client %d.\n", i);
            clients[i].waitingForAck = false;
            any = true;
            clients[i].recording = (ACK_MSG == ACK_STARTED);
        } else if (((timestamps.now - clients[i].commandSent) > TIMEOUT_MS)){
            Serial.printf("[HOST] ACK timeout for client %d.\n", i);
            clients[i].waitingForAck = false;

        } else all = false;
    }
    return (any || all);
}

// ── Client callbacks ──────────────────────────────────────────────────────────
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        // find which index this client is
        for (int i = 0; i < knownMacCount; i++) {
            if (clients[i].pClient == pClient) {
                Serial.printf("[HOST] Client %d connected.\n", i);
                return;
            }
        }
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        for (int i = 0; i < knownMacCount; i++) {
            if (clients[i].pClient == pClient) {
                Serial.printf("[HOST] Client %d disconnected! Reason: %d\n", i, reason);
                // Clear characteristics — they're invalid after disconnect
                clients[i].pCmd        = nullptr;
                clients[i].pAck        = nullptr;
                clients[i].pBatt       = nullptr;
                clients[i].ackReceived = false;
                clients[i].battPercent = 255;
                clients[i].connected   = false;
                clients[i].nextTry     = timestamps.now;
                // Don't delete pClient here — do it when reconnecting
                return;
            }
        }
    }
};

static ClientCallbacks clientCallbacks; 
bool connectToClient(int idx) {
     
    Serial.printf("[HOST] directly connecting to %s...\n", knownMacs[idx]);

    NimBLEClient* pC = NimBLEDevice::createClient();
        pC->setClientCallbacks(&clientCallbacks, false);  // false = don't delete on disconnect
        pC->setConnectTimeout(2000);  // short timeout — don't wait long

    if (!pC->connect(NimBLEAddress(knownMacs[idx], BLE_ADDR_PUBLIC))) {
        Serial.printf("[HOST] Direct connect failed: %s\n", knownMacs[idx]);
        NimBLEDevice::deleteClient(pC);
        return false;
    }
    Serial.printf("[HOST] Connected to: %s\n", knownMacs[idx]);

    // ── CMD / ACK service ─────────────────────────────────────────────────────
    NimBLERemoteService* pSvc = pC->getService(SERVICE_UUID);
    if (!pSvc) {
        Serial.printf("[HOST] Service not found on %s\n", knownMacs[idx]);
        pC->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic* pCmd = pSvc->getCharacteristic(CMD_CHAR_UUID);
    NimBLERemoteCharacteristic* pAck = pSvc->getCharacteristic(ACK_CHAR_UUID);
    if (!pCmd || !pAck) {
        Serial.printf("[HOST] Characteristics not found on %s\n", knownMacs[idx]);
        pC->disconnect();
        return false;
    }

    clients[idx].pClient = pC;
    clients[idx].pCmd    = pCmd;
    clients[idx].pAck    = pAck;

    // Subscribe to ACK notifications
    pAck->subscribe(true,
        [idx](NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
            if (!len) return;
            Serial.printf("[HOST] Client %d ACK: 0x%02X\n", idx, data[0]);
            clients[idx].ackReceived = true;
            clients[idx].ackValue = data[0];
        }
    );

    // ── Battery service (optional) ────────────────────────────────────────────
    NimBLERemoteService* pBattSvc = pC->getService(BATTERY_SERVICE_UUID);
    if (pBattSvc) {
        NimBLERemoteCharacteristic* pBatt =
            pBattSvc->getCharacteristic(BATTERY_CHAR_UUID);
        if (pBatt) {
            clients[idx].pBatt       = pBatt;
            clients[idx].battPercent = pBatt->readValue<uint8_t>();
            Serial.printf("[HOST] Client %d battery: %d%%\n",
                          idx, clients[idx].battPercent);

            pBatt->subscribe(true,
                [idx](NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
                    if (!len) return;
                    clients[idx].battPercent = data[0];
                    Serial.printf("[HOST] Client %d battery update: %d%%\n",
                                  idx, clients[idx].battPercent);
                }
            );
        }
    }
    clients[idx].connected = true;
    Serial.printf("[HOST] Client %d fully connected.\n", idx);
    return true;

}
void callConnectToClient(int idx){
        if (timestamps.now<clients[idx].nextTry || clients[idx].connected) return;
        clients[idx].connected = connectToClient(idx);
        if (!clients[idx].connected) clients[idx].nextTry =timestamps.now +  NextTryDelta;
}
void ConnectAll(){
    static unsigned long lastCheck = 0;
    if (timestamps.now - lastCheck < 500) return;
    static bool connecting = false;
    if (connecting) return;
    lastCheck = timestamps.now;
    for (int i = 0; i < knownMacCount; i++) {

        if (clients[i].pClient && !clients[i].pClient->isConnected()) {
            Serial.printf("[HOST] Client %d lost, reconnecting...\n", i);
            NimBLEDevice::deleteClient(clients[i].pClient);
            clients[i].pClient = nullptr;
            clients[i].connected = false;
            clients[i].nextTry = timestamps.now-1;
        }
        
        if (!clients[i].connected &&
            timestamps.now >= clients[i].nextTry) {
            connecting = true;
            clients[i].connected = connectToClient(i);
            connecting = false;
            if (!clients[i].connected)
                clients[i].nextTry = timestamps.now + NextTryDelta;
            // Only one reconnect attempt per loop cycle
            return;
        }
    }
}

void battery(int x0, int soc){
    int h = max(min(soc,100), 0) * 28 / 100;
    display.drawRect(x0, 3, 16, 29);
    display.fillRect(x0+4,0,8,4);
    display.fillRect(x0+1,31-h,14,h);
    if ((soc < 20) && blink){
        display.drawRect(x0+7, 20, 2,2);
        display.drawRect(x0+7,8,2,10);
    }

}
void phone(bool charging, int x0){
    display.drawRect(x0,0,16,32);
    display.clearPixel(x0,0);
    display.clearPixel(x0+15,0);
    display.clearPixel(x0,31);
    display.clearPixel(x0+15,31);
    if (charging){

        display.fillTriangle(x0+8,5,x0+4,16,x0+8,16);
        display.fillTriangle(x0+8,27,x0+12,16,x0+8,16);

    }
    // if (!charging){
    //     int angle = (now/100) % 360;
    //     float local_cos = cos(angle * PI / 180);
    //     float local_sin = sin(angle * PI / 180);
    //     display.fillTriangle(
    //         int(local_cos*6)+x0+8,int(local_sin*6) + 15,
    //         int(local_sin*4)+x0+8,int(local_cos*4) + 15,
    //         -int(local_sin*4)+x0+8,-int(local_cos*4) + 15

    //     );
    // }

}

void camera(int xs, int n, bool connected, bool rec, int soc){
    int x0 = 26*(n/ 2) + xs;
    int y0 = 16*(n%2);

        display.drawRect(x0,y0+6,20,9);
        display.drawCircle(x0+5,y0+3,3);
        display.drawCircle(x0+13,y0+3,3);
        display.drawTriangle(x0+19,y0+10,x0+23,y0+6,x0+23,y0+14);
        if (!connected){
            display.drawLine(x0,y0,x0+23,y0+15);
            display.drawLine(x0,y0+15,x0+23,y0);
        } else{
             int w = max(min(soc,100), 0) * 18 / 100;
            display.fillRect(x0+1,y0+7,w,7);if (rec){
            display.fillTriangle(x0+19,y0+10,x0+23,y0+6,x0+23,y0+14);
            if (blink) display.fillCircle(x0+5,y0+3,3);
            else display.fillCircle(x0+13,y0+3,3);
        }
        } 

}

void GPS(int x0, bool powered){
    int r = 10;
    display.fillRect(x0,r-1,2*r,32-2*r);
    display.fillRect(x0,0,2,r);
    x0 += r;
    display.fillCircle(x0,r-1,r);
    display.fillCircle(x0,32-r,r);
    display.setColor(BLACK);
    display.fillRect(x0-r+2,r-4,2*r-4,32-2*r+7);
    display.setColor(WHITE);
    if (powered){
        // display.fillTriangle(x0,5,x0-4,16,x0,16);
        // display.fillTriangle(x0,27,x0+4,16,x0,16);
    } else {
        // display.drawLine(x0-r+2,r-4,x0+r-2,36-r);
        // display.drawLine(x0-r+2,36-r,x0+r-2,r-4);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(x0,16,"z");
        display.drawString(x0-4,20,"z");
        display.drawString(x0+4,10,"Z");
    }
}


void cleardisp(int startx, int starty, int lx, int ly)
{
  display.setColor(BLACK);
  display.fillRect(startx, starty, lx, ly);
  display.setColor(WHITE);
}
void cleardisp(void)
{
  cleardisp(0, 0, 128, 32);
}
typedef enum {LEFT, CENTER, RIGHT} ALIGN;

void displine(int line, String text,  OLEDDISPLAY_TEXT_ALIGNMENT align = TEXT_ALIGN_LEFT, int size = 10, bool clear_line = false)
{

  int start = 0;
  int cx1 = 0;
  switch (size)
  {
  case 10:
    display.setFont(ArialMT_Plain_10);
    break;
  case 16:
    display.setFont(ArialMT_Plain_16);
    break;
  case 24:
    display.setFont(ArialMT_Plain_24);
    break;
  default:
    display.setFont(ArialMT_Plain_10);
    break;
  }
  int strwidth = display.getStringWidth(text);
  switch (align)
  {
  case TEXT_ALIGN_LEFT:
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    cx1 = start;
    break;
  case TEXT_ALIGN_CENTER:
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    start = 63;
    cx1 = start - strwidth / 2;
    break;
  case TEXT_ALIGN_RIGHT:
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    start = 127;
    cx1 = start - strwidth;
    break;
  default:
    break;
  }
  if (clear_line)
  {
    cleardisp(0, line * size, 127, size);
  }
  else
  {
    cleardisp(cx1, line * size, strwidth, size);
  }
  display.drawString(start, line * size, text);
  display.setFont(ArialMT_Plain_10);
}

void changeState(String prevState, String nextState, unsigned long& ts, String ID){
    ts = timestamps.now;
    updateDisplay=true;
    timestamps.display = timestamps.now;
    if (prevState == nextState) {
        Serial.println("[HOST] ["+ ID +"] "+ prevState + " requested again.");

        return;
    }
    Serial.println("[HOST] ["+ ID +"] Going from "+ prevState + " to " + nextState);

}

void turnDisplayOff(){
    
    display.displayOff();
    displayReady = false;
}
void endI2C(){

    Wire.end();                        // release SDA/SCL
    pinMode(SDA, INPUT);               // avoid phantom current through I2C pins
    pinMode(SCL, INPUT);
}
void turnDisplayOn(){if (!displayReady) displayReady = display.init();}

void displayState(){
    if (displayReady) {
        displine(0, ShortSystemStateString[sysState], TEXT_ALIGN_CENTER, 16, true);
        // displine(2, ShortCameraStateString[camState], TEXT_ALIGN_RIGHT, 10, false);
        display.display();
    }
}
void displayCamera(){
    if (displayReady){
        display.clear();
        battery(112, stateOfChargeMain);
        phone(PhoneCharging==CHARGING, 90);
        GPS(64,((timestamps.now+3000)%10000)<5000);

        for (int i=0;i<knownMacCount;i++){
            camera(0, i, clients[i].connected, clients[i].recording,clients[i].battPercent);
        }

        display.display();
    }
}
void displayMenu(){

}
void changeState(DisplayState nextState){
    changeState(DisplayStateString[dispState], DisplayStateString[nextState],timestamps.display, "DISPLAY");
    if ((dispState == DISPLAY_OFF) && (nextState != DISPLAY_OFF)) {
        turnDisplayOn();
    }
    if (((dispState == DISPLAY_OFF) || (dispState == DISPLAY_SYS_STATE)) && nextState==DISPLAY_CAMERA){timestamps.display_timeout=timestamps.now;}        
    cleardisp();

    dispState = nextState;
}
void changeState(SystemState nextState){
    changeState(SystemStateString[sysState], SystemStateString[nextState],timestamps.system, "SYSTEM");
    sysState = nextState;
    changeState(DISPLAY_SYS_STATE);
}
void changeState(CameraState nextState){
    changeState(CameraStateString[camState], CameraStateString[nextState],timestamps.camera, "CAMERA");
    camState = nextState;
}
void changeState(PhoneState nextState){
    changeState(PhoneStateString[PhoneCharging], PhoneStateString[nextState],timestamps.phone, "PHONE");
    PhoneCharging = nextState;
}
void updateDisplayState(){
    unsigned long windowsize = timestamps.now - timestamps.display;
    blink = (timestamps.now%DISPLAY_BLINK_RATE)<(DISPLAY_BLINK_RATE/2);
    if (prevDispState!=dispState) {updateDisplay = true;}
    switch (dispState){
        case DISPLAY_SYS_STATE:{
            if (updateDisplay){displayState();break;}
            if (windowsize > 3000) {
                changeState(DISPLAY_CAMERA); 
                timestamps.display_timeout=timestamps.now;
                return; 
                break;
            }
            break;
        }
        case DISPLAY_CAMERA:{
            if (updateDisplay){displayCamera();break;}
            if (windowsize>=DISPLAY_REFRESH_RATE){displayCamera();timestamps.display+=DISPLAY_REFRESH_RATE;break;}
            if ((timestamps.now - timestamps.display_timeout) > 30000) {changeState(DISPLAY_OFF); return; break;}
            break;
        }
        case DISPLAY_OFF:{
            if (updateDisplay){
                turnDisplayOff();
                // cleardisp();
                // display.display();
                // display.clear();
                break;
            };
            if (KeypadMain.anyPress){
                // turnDisplayOn();
                timestamps.display_timeout=timestamps.now;
                changeState(DISPLAY_CAMERA);
                return;
                break;
            };
            break;
        }
        case DISPLAY_SETUP:{
            if (updateDisplay){displayMenu();break;}
            if (windowsize>=DISPLAY_REFRESH_RATE){displayMenu();timestamps.display+=DISPLAY_REFRESH_RATE;break;}
            break;
        }
    }
    updateDisplay = false;
    prevDispState = dispState;

}

void togglePDBoard(){
    switch (PhoneCharging){
        case CHARGING:{
            // one button press to initiate charging
            break;
        }
        case DISCHARGING:{
            // three button presses to turn off PD Board
            break;
        }
    }
}

void updatePhoneState(){
    unsigned long window = timestamps.now - timestamps.phone;
    PhoneState nextPhoneState;
    switch (PhoneCharging){
        case DISCHARGING:{nextPhoneState = CHARGING; break;}
        case CHARGING:{nextPhoneState = DISCHARGING; break;}
        default:{nextPhoneState = DISCHARGING; break;}
    }
    if (toggleCharging){
        if (window > (1000*10) || !ChargerInitialized){
            ChargerInitialized = true;
            changeState(nextPhoneState);
        } else {
            changeState(PhoneCharging);

        }
        togglePDBoard();
    }
    toggleCharging = false;
}


void XCTrack(){
    Serial.println("[HOST] XCTrack");
    bleKeyboard.print("xctrack");
    toggleCharging=true;
}

void ALT_TAB(){
    Serial.println("[HOST] Alt + Tab");
    bleKeyboard.press(KEY_LALT);
    bleKeyboard.press(KEY_TAB);
    bleKeyboard.releaseAll();
}

void Toggle_Recording(){
    if (recording) {
        Serial.println("[HOST] Stop Recording");
        recording = false;
    }
    else {
        
        Serial.println("[HOST] Start Recording");
        recording = true;
    }
}

void updateGPSState(){}

void updateCameraState(){
    static NimBLEScan* pScan = nullptr;
    switch (camState){
        case CAMERA_DISCONNECTED:{
            if ((timestamps.now - timestamps.camera)>2000) {

                Serial.println("[HOST] Starting client scan...");
                changeState(CAMERA_CONNECTING);
            }
            break;}
        // case CAMERA_SCANNING:{
        //             // Scan still running — check if it's done
        //     if ((timestamps.now - timestamps.camera)>2000) 
        //     {
        //     if (pScan->isScanning()) break;

        //     Serial.println("[HOST] Scan complete.");
        //     changeState(CAMERA_CONNECTING);}
        //     break;}
        case CAMERA_CONNECTING:{
            ConnectAll();
            changeState(CAMERA_READY);
            changeState(RUNNING);
            break;}
        case CAMERA_READY:{
            waitForAck(ACK_STOPPED);
            if (recording) {
                sendCmd(CMD_START);
                changeState(CAMERA_BOOTING);
            }
            break;}
        case CAMERA_BOOTING:{
            if (waitForAck(ACK_STARTED)) {
                ConnectAll();
                
                bleKeyboard.tap(KEY_R);
                changeState(CAMERA_RECORDING);
            }
            break;}
        case CAMERA_RECORDING:{
            waitForAck(ACK_STARTED);
            if (!recording) {
                sendCmd(CMD_STOP);
                changeState(CAMERA_SHUTDOWN);
            }
            break;}
        case CAMERA_SHUTDOWN:{
            if (waitForAck(ACK_STOPPED)) {
                bleKeyboard.tap(KEY_R);
                changeState(CAMERA_READY);
            }
            break;}
        default:
            break;
    }
}

void sendKey( char KEY){
    if (KEY == 0) return;
    if (KEY == XCT) XCTrack();
    else if (KEY == AltTab) ALT_TAB();
    else if (KEY == REC) Toggle_Recording();
    else {
        // Serial.print(String(KEY) + " ");
        if (key_chars.indexOf(String(KEY))>-1)bleKeyboard.tap(key_array[key_chars.indexOf(String(KEY))]);
        else if (media_chars.indexOf(String(KEY))>-1)bleKeyboard.tap(media_array[media_chars.indexOf(String(KEY))]);
        else bleKeyboard.write((uint8_t)KEY);
    }
}
void sendKeys(){
    if (!keyboardAvailable) return;
    char* keys = KeypadMain.getKeys();
    anyKey = false;
    for (byte i = 0; i<(ROWS*COLS); i++){
        if (keys[i] == 0) continue;
        anyKey = true;
        sendKey(keys[i]);
        keys[i] = char(0);
    }
}

typedef enum {MAIN, POWER} KeyPadType;

void setupKeypad(KeyPadType kpt){
    switch (kpt){
        case MAIN:{
            KeypadMain.init(makeKeymap(KEYS), makeKeymap(ALT_KEYS),(uint8_t*)row_GPIOs, (uint8_t*)col_GPIOs, ROWS, COLS, no_repeat);
            break;
        }
        case POWER:{
            KeypadPower.init((uint8_t*)power_row_GPIOs, power_col_GPIO, 2);
            break;
        }
    }
}

void enterDeepSleep(bool wait=true) {
    
    Serial.println("[HOST] Shutdown confirmed");
    Serial.println("[HOST] Entering Deep Sleep"); 
    Serial.println("[HOST] Ending BLEKeyboard");
    // Shut down Bluetooth cleanly first
    bleKeyboard.end();
    changeState(DISCHARGING);
    togglePDBoard();
    if (wait) delay(2000);
    turnDisplayOff();
    endI2C();
    // Small delay to let BT stack finish shutting down
    delay(100);

    Serial.println("[HOST] prepare Pins for Wake Up");

  for (int r=0; r< ROWS;r++) {
    uint8_t GPIO = row_GPIOs[r];
    if (GPIO == power_row_GPIO){
        pinMode(GPIO, OUTPUT);
        digitalWrite(GPIO, LOW);
        gpio_hold_en(gpio_num_t(GPIO));
    } else {

    pinMode(row_GPIOs[r], INPUT);
    }
  }

  for (int c=0; c<COLS; c++) pinMode(col_GPIOs[c], INPUT);

  esp_deep_sleep_enable_gpio_wakeup(BIT(power_col_GPIO), ESP_GPIO_WAKEUP_GPIO_LOW);

  Serial.println("[HOST] Serial Flush");
  Serial.flush();
  delay(100);
  Serial.println("[HOST] gpio_deep_sleep_hold_en");
  gpio_deep_sleep_hold_en();
  delay(100);
  Serial.println("[HOST] Going to sleep now");
  esp_deep_sleep_start();
}
void updateSystemState(){
    unsigned long windowsize = timestamps.now - timestamps.system;
    switch (sysState){
        case OFF:{
            setupKeypad(POWER);
            // turnDisplayOn();
            KeypadPower.readKey(0);
            changeState(INITIAL_BOOT);
            changeState(CAMERA_OFF);
            break;
        }

        case INITIAL_BOOT:{
            timestamps.display = timestamps.now;
            KeypadPower.readKey(0);
            if (windowsize > (POWER_CYCLE_DELAY/2)) changeState(WAIT_FOR_BUTTON_HOLD);
            break;
        }
        case WAIT_FOR_BUTTON_HOLD:{
            timestamps.display = timestamps.now;
            KeypadPower.readKey(0);
            if (KeypadPower.buttonState) {
                changeState(WAIT_FOR_BUTTON_RELEASE);
                timestamps.system -= POWER_CYCLE_DELAY/2;
                Serial.println("[HOST] Waiting for Release of Power Button");
                break;
            }
            if (windowsize>=POWER_CYCLE_DELAY) {
                changeState(SHUTDOWN);
                Serial.println("[HOST] Power Button Push not detected");
                break;
            }
            break;
        }
        case WAIT_FOR_BUTTON_RELEASE:{
            timestamps.display = timestamps.now;
            KeypadPower.readKey(0);
            if (KeypadPower.keyState==RELEASED){
                changeState(WAIT_FOR_CONFIRMATION);
                Serial.println("[HOST] First key released — waiting for second key");
                break;
                KeypadPower.readKey(1);
            }
            break;  
        }
        case WAIT_FOR_CONFIRMATION:{
            timestamps.display = timestamps.now;
            KeypadPower.readKey(1);
            if (windowsize > POWER_CYCLE_DELAY) {
                changeState(SHUTDOWN);
                Serial.println("[HOST] Second key pressed too late — going back to sleep");
                break;
            }
            if (KeypadPower.stateChanged){
                changeState(SETUP);
                Serial.println("[HOST] Startup Confirmed");
                break;
            }
            break;
        }
        case SETUP:{
            Serial.println("[HOST] Boot number: " + String(bootCount));
            Serial.printf("[HOST] Free heap before BLE init: %d bytes\n", ESP.getFreeHeap());
            Serial.println("[HOST] Starting BLEKeyboard");

            bleKeyboard.setDebugLevel(HIDLogLevel::Normal);
            bleKeyboard.setKeyGap(1);
            bleKeyboard.setTapDelay(10);
            bleKeyboard.begin();
            Serial.printf("[HOST] Free heap after BLE init: %d bytes\n", ESP.getFreeHeap());
            NimBLEDevice::setPower(Max_TX_Power_db);   // max TX power
            Serial.printf("[HOST] MAC: %s\n", NimBLEDevice::getAddress().toString().c_str());
            setupKeypad(MAIN);
            delay(1000);
            keyboardAvailable = true;
            changeState(RUNNING);
            changeState(CAMERA_CONNECTING);
            toggleCharging = true;
            KeypadPower.setKey(0);
            break;
        }
        case RUNNING:{
            sendKeys();
            KeypadPower.readKey(0);
            if (KeypadPower.veryLongPress(POWER_CYCLE_DELAY)){
                changeState(INITIAL_SHUTDOWN);
                Serial.println("[HOST] Waiting for Release of Power Button");
            }
            break;

        }
        case INITIAL_SHUTDOWN:{
            KeypadPower.readKey(0);
            timestamps.display = timestamps.now;
            if (KeypadPower.keyState == RELEASED){
                changeState(WAIT_FOR_SHUTDOWN_CONFIRMATION);
                Serial.println("[HOST] Waiting for Confirmation Button");
                for (int i = 0; i<10; i++){
                    sendKeys();
                    KeypadPower.readKey(1);
                    delay(10);
                }
            }
            break;
        }
        case WAIT_FOR_SHUTDOWN_CONFIRMATION:{
            sendKeys();
            KeypadPower.readKey(1);
            timestamps.display = timestamps.now;
            if (windowsize<200) break;
            if ((windowsize) > POWER_CYCLE_DELAY){
                changeState(RUNNING);
                KeypadPower.setKey(0);
                Serial.println("[HOST] Confirmation didn't happen in time");
                break;
            } 
            if (!KeypadPower.buttonState && KeypadPower.stateChanged){
                changeState(SHUTDOWN);
                Serial.println("[HOST] Shutdown confirmed");
                break;
            };
            if (anyKey){
                changeState(RUNNING);
                KeypadPower.setKey(0);
                Serial.println("[HOST] Shutdown canceled by any key");
                break;
            }
            break;
        }
        case SHUTDOWN:{
            enterDeepSleep();
            break;
        }
    }
}
void validate_wake_up_reason(){
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_GPIO){
        enterDeepSleep(false);
    }
}
void setup() {
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(gpio_num_t(power_row_GPIO));
    // validate_wake_up_reason();
    Serial.begin(BAUD);
    delay(INITIAL_DELAY);
    Serial.printf("[HOST] Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[HOST] Min free heap: %d bytes\n", ESP.getMinFreeHeap());
    Serial.printf("[HOST] knownMacCount: %d\n", knownMacCount);
    Serial.println("[HOST] Starting...");
    ++bootCount;
}


void loop() {
    timestamps.now = millis();
    updateGPSState();
    updateSystemState();
    updateCameraState();
    updatePhoneState();
    updateDisplayState();
    delay(10);
}  