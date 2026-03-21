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
    unsigned long display;
} timestamps;

typedef enum {INITIAL_BOOT,WAIT_FOR_BUTTON_HOLD, WAIT_FOR_TIME, WAIT_FOR_BUTTON_RELEASE, WAIT_FOR_CONFIRMATION, SETUP, RUNNING, INITIAL_SHUTDOWN, WAIT_FOR_SHUTDOWN_CONFIRMATION, SHUTDOWN, OFF} SystemState;
const String SystemStateString[] = {"INITIAL BOOT", "WAIT FOR BUTTON HOLD","WAIT FOR TIME", "WAIT FOR BUTTON RELEASE", "WAIT FOR CONFIRMATION", "SETUP", "RUNNING", "INITIAL SHUTDOWN", "WAIT FOR SHUTDOWN CONFIRMATION", "SHUTDOWN", "OFF"};
const String ShortSystemStateString[] = {"HOLD", "HOLD","HOLD", "RELEASE", "CONFIRM", "SETUP", "RUNNING", "RELEASE", "CONFIRM", "SHUTDOWN", "OFF"};
SystemState sysState = OFF;

typedef enum  {DISPLAY_OFF, DISPLAY_SYS_STATE, DISPLAY_CAMERA} DisplayState;
const String DisplayStateString[] = {"OFF", "SYS_STATE", "CAMERA"};
DisplayState dispState = DISPLAY_OFF;
DisplayState prevDispState = DISPLAY_OFF;
bool updateDisplay = false;

bool anyKey = false;


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
#define REC ((char) 'r')
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


Keypad KeypadMain;
Keypad KeypadPower;


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
}

void changeState(String prevState, String nextState, unsigned long& ts, String ID){
    ts = timestamps.now;
    if (prevState == nextState) return;
    Serial.println("[HOST] ["+ ID +"] Going from "+ prevState + " to " + nextState);

}
void displayState(){
    if (displayReady) {
        displine(0, ShortSystemStateString[sysState], TEXT_ALIGN_CENTER, 16, true);
        displine(2, ShortCameraStateString[camState], TEXT_ALIGN_RIGHT, 10, false);
        display.display();
    }
}
void displayCamera(){
    if (displayReady){
        displine(0,"CAMERA");
        display.display();
    }
}
void changeState(DisplayState nextState){
    changeState(DisplayStateString[dispState], DisplayStateString[nextState],timestamps.display, "DISPLAY");
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

void turnDisplayOff(){
    
    display.displayOff();
    Wire.end();                        // release SDA/SCL
    pinMode(SDA, INPUT);               // avoid phantom current through I2C pins
    pinMode(SCL, INPUT);
    displayReady = false;
}
void turnDisplayOn(){if (!displayReady) displayReady = display.init();}

void updateDisplayState(){
    if (dispState!=prevDispState) updateDisplay = true;
    switch (dispState){
        case (DISPLAY_SYS_STATE):{
            if ((timestamps.now - timestamps.display) > 3000) {changeState(DISPLAY_CAMERA); break;}
            if (updateDisplay){displayState();break;}
            break;
        }
        case (DISPLAY_CAMERA):{
            if ((timestamps.now - timestamps.display) > 10000) changeState(DISPLAY_OFF);
            if (updateDisplay){displayCamera();break;}
            break;
        }
        case (DISPLAY_OFF):{
            if (updateDisplay){
                // turnDisplayOff();
                display.clear();
                break;
            };
            if (anyKey){
                turnDisplayOn();
                changeState(DISPLAY_CAMERA);
                break;
            };
            break;
        }
    }
    updateDisplay = false;
    prevDispState = dispState;

}


void XCTrack(){
    Serial.println("[HOST] XCTrack");
    bleKeyboard.print("xctrack"); // TODO check if it still works
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

void updateRecordingState(){
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
    if (wait) delay(2000);
    turnDisplayOff();
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
            turnDisplayOn();
            KeypadPower.readKey(0);
            changeState(INITIAL_BOOT);
            changeState(CAMERA_OFF);
            break;
        }

        case INITIAL_BOOT:{
            KeypadPower.readKey(0);
            if (windowsize > (POWER_CYCLE_DELAY/2)) changeState(WAIT_FOR_BUTTON_HOLD);
            break;
        }
        case WAIT_FOR_BUTTON_HOLD:{
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
            if (KeypadPower.keyState == RELEASED){
                changeState(WAIT_FOR_SHUTDOWN_CONFIRMATION);
                Serial.println("[HOST] Waiting for Confirmation Button");
                for (int i = 0; i<10; i++){
                    KeypadPower.readKey(1);
                    delay(10);
                }
            }
            break;
        }
        case WAIT_FOR_SHUTDOWN_CONFIRMATION:{
            sendKeys();
            KeypadPower.readKey(1);
            if (windowsize<200) break;
            if (anyKey){
                changeState(RUNNING);
                KeypadPower.setKey(0);
                Serial.println("[HOST] Shutdown canceled by any key");
                break;
            }
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
            break;
        }
        case SHUTDOWN:{
            enterDeepSleep();
            break;
        }
    }
}
// void shutdown(){
//     // if (KeypadMain.anyPress){
//     //     changeState(RUNNING);
//     //     return;

//     // }
//     KeypadPower.readKey();
//     switch (sysState){

//         case RUNNING:
//     }
// }

// void validate_wake_up_sequence(){

//     unsigned long windowStart = millis();
//     while ((millis()-windowStart)<(POWER_CYCLE_DELAY/2)){delay(10);}
//     changeState(WAIT_FOR_BUTTON_HOLD);
//     KeypadPower.readKey(0);
//     while (true){
//         KeypadPower.readKey();
//         int windowsize = millis() - windowStart;
//         switch (sysState){
//             case WAIT_FOR_BUTTON_HOLD:
//                 if (KeypadPower.buttonState) {//} || (windowsize > POWER_CYCLE_DELAY*3/4)) {
//                     windowStart = millis() - POWER_CYCLE_DELAY/2;
//                     changeState(INITIAL_BOOT);
//                     Serial.println("[HOST] Waiting for Release of Power Button");
//                 } else if (windowsize>=POWER_CYCLE_DELAY) changeState(SHUTDOWN);
//                 break;
//             case INITIAL_BOOT:
//                 if (windowsize>=POWER_CYCLE_DELAY) {
//                     changeState(WAIT_FOR_BUTTON_RELEASE);
//                 }   else if (KeypadPower.keyState==RELEASED){
//                         changeState(SHUTDOWN);
//                         Serial.println("[HOST] First key released too early — going back to sleep");
//                 }
//                 break;
                
//             case WAIT_FOR_BUTTON_RELEASE:
//                 if (KeypadPower.keyState==RELEASED){
//                     changeState(WAIT_FOR_CONFIRMATION);
//                     KeypadPower.readKey(1);
//                     Serial.println("[HOST] First key released — waiting for second key");
//                     windowStart = millis();
//                 }
//                 break;
//             case WAIT_FOR_CONFIRMATION:
//                 if (windowsize > POWER_CYCLE_DELAY) {
//                     changeState(SHUTDOWN);
//                     Serial.println("[HOST] Second key pressed too late — going back to sleep");
//                 } else if (KeypadPower.stateChanged){
//                     changeState(SETUP);
//                 }
//                 break;
//             case SHUTDOWN:
//                 Serial.println("[HOST] invalid boot sequence");
//                 enterDeepSleep();
//                 return;
//                 break;
//             case SETUP:
//                 if (KeypadPower.buttonState==IDLE) return;
//                 break;
//         }
//         if (windowsize > 10000) changeState(SHUTDOWN);
//         delay(10);
        
//     }
// }
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
    updateSystemState();
    // bleKeyboard.setBatteryLevel(75);
    // updateRecordingState();
    // ConnectAll();
    // sendKeys();
    // shutdown();
    updateDisplayState();
    delay(10);
}  