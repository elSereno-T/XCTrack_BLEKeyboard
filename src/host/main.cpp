#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HijelHID_BLEKeyboard.h>
#include <Keypad.h>
#include <shared.h>


struct {
    unsigned long now;
    unsigned long camera;
    unsigned long system;
} timestamps;


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

typedef enum {INITIAL_BOOT,WAIT_FOR_BUTTON_HOLD, WAIT_FOR_TIME, WAIT_FOR_BUTTON_RELEASE, WAIT_FOR_CONFIRMATION, SETUP, RUNNING, INITIAL_SHUTDOWN, SHUTDOWN, OFF} SystemState;
const String SystemStateString[] = {"INITIAL BOOT", "WAIT FOR BUTTON HOLD","WAIT FOR TIME", "WAIT FOR BUTTON RELEASE", "WAIT FOR CONFIRMATION", "SETUP", "RUNNING", "INITIAL SHUTDOWN", "SHUTDOWN", "OFF"};
SystemState sysState = OFF;




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

// const uint16_t DEBOUNCE_MS  = 50;
// const uint16_t HOLD_TIME = 500;
// const uint16_t REPEAT_DELAY = 200;
// const uint16_t REPEAT_ACCELERATION  = 20;
// const uint16_t REPEAT_MAX_RATE  = 50;
const uint16_t POWER_CYCLE_DELAY = 3000;

HijelHID_BLEKeyboard bleKeyboard("XCTrack Keypad", "TS", 50);


// unsigned long now;


bool recording = false;
unsigned long pre_shutdown_release = 0;

RTC_DATA_ATTR int bootCount = 0;


#include <Wire.h>
#include <SSD1306Wire.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);

bool displayReady=false;


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
void displine(int line, String text, char align = 'l', int size = 10, bool clear_line = false)
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
  case 'l':
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    cx1 = start;
    break;
  case 'c':
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    start = 63;
    cx1 = start - strwidth / 2;
    break;
  case 'r':
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

void hello_world(){
    if (!displayReady) return;
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0,0,"Hello world!");
    display.display();

}
void changeState(String prevState, String nextState, byte line, unsigned long& ts, byte size=10){
    if (prevState == nextState) return;
    ts = timestamps.now;
    Serial.println("Going from "+ prevState + " to " + nextState);
    if (displayReady) {
        displine(line, nextState, 'c', size, true);
        display.display();
    }

}
void changeState(SystemState nextState){
    changeState(SystemStateString[sysState], SystemStateString[nextState],0, timestamps.system);
    sysState = nextState;
}
void changeState(CameraState nextState){
    changeState(CameraStateString[camState], CameraStateString[nextState],1, timestamps.camera);
    camState = nextState;
}


void XCTrack(){
    Serial.println("XCTrack");
    bleKeyboard.print("XCTrack");
}

void ALT_TAB(){
    Serial.println("Alt + Tab");
    bleKeyboard.press(KEY_LALT);
    bleKeyboard.press(KEY_TAB);
    bleKeyboard.releaseAll();
}

void Toggle_Recording(){
    if (recording) {
        Serial.println("Stop Recording");
        recording = false;
    }
    else {
        
        Serial.println("Start Recording");
        recording = true;
    }
}

void RecordingState(){
    static NimBLEScan* pScan = nullptr;
    switch (camState){
        case CAMERA_DISCONNECTED:{
            if ((timestamps.now - timestamps.camera)>2000) {

                Serial.println("[HOST] Starting client scan...");
                changeState(CAMERA_SCANNING);
            }
            break;}
        case CAMERA_SCANNING:{
                    // Scan still running — check if it's done
            if ((timestamps.now - timestamps.camera)>2000) 
            {
            if (pScan->isScanning()) break;

            Serial.println("[HOST] Scan complete.");
            changeState(CAMERA_CONNECTING);}
            break;}
        case CAMERA_CONNECTING:{
            changeState(CAMERA_READY);
            break;}
        case CAMERA_READY:{
            if (recording) changeState(CAMERA_BOOTING);
            break;}
        case CAMERA_BOOTING:{
            if ((timestamps.now - timestamps.camera)>2000) changeState(CAMERA_RECORDING);
            break;}
        case CAMERA_RECORDING:{
            if (!recording) changeState(CAMERA_SHUTDOWN);
            break;}
        case CAMERA_SHUTDOWN:{
            if ((timestamps.now - timestamps.camera)>2000) changeState(CAMERA_READY);
            break;}
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
    char* keys = KeypadMain.getKeys();
    for (byte i = 0; i<(ROWS*COLS); i++){
        if (keys[i] == 0) continue;
        sendKey(keys[i]);
        keys[i] = char(0);
    }
}



void setupKeypad(bool mainKB){
    if (mainKB){
        KeypadMain.init(makeKeymap(KEYS), makeKeymap(ALT_KEYS),(uint8_t*)row_GPIOs, (uint8_t*)col_GPIOs, ROWS, COLS, no_repeat);
    } else {
        KeypadPower.init((uint8_t*)power_row_GPIOs, power_col_GPIO, 2);
    }
}

void enterDeepSleep() {
    
    changeState(SHUTDOWN);
    Serial.println("Shutdown confirmed");
    Serial.println("Entering Deep Sleep"); 
    Serial.println("Ending BLEKeyboard");
    // Shut down Bluetooth cleanly first
    bleKeyboard.end();
    display.displayOff();
    Wire.end();                        // release SDA/SCL
    pinMode(SDA, INPUT);               // avoid phantom current through I2C pins
    pinMode(SCL, INPUT);

    // Small delay to let BT stack finish shutting down
    delay(100);

    Serial.println("prepare Pins for Wake Up");

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

  Serial.println("Serial Flush");
  Serial.flush();
  delay(100);
  Serial.println("gpio_deep_sleep_hold_en");
  gpio_deep_sleep_hold_en();
  delay(100);
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
}
void shutdown(){
    // if (KeypadMain.anyPress){
    //     changeState(RUNNING);
    //     return;

    // }
    KeypadPower.readKey();
    switch (sysState){

        case RUNNING:
            if (KeypadPower.veryLongPress(POWER_CYCLE_DELAY)){
                changeState(INITIAL_SHUTDOWN);
                Serial.println("Waiting for Release of Power Button");
            }
            break;
        case INITIAL_SHUTDOWN:if (KeypadPower.keyState == RELEASED){
                pre_shutdown_release = timestamps.now;
                changeState(WAIT_FOR_CONFIRMATION);
                Serial.println("Waiting for Confirmation Button");
                KeypadPower.readKey(1);
            }
            break;
        case WAIT_FOR_CONFIRMATION:
            if ((timestamps.now - pre_shutdown_release) > POWER_CYCLE_DELAY){
                changeState(RUNNING);
                KeypadPower.setKey(0);
                Serial.println("Confirmation didn't happen in time");
            } else if (KeypadPower.stateChanged){
                enterDeepSleep();
            }
    }
}

void validate_wake_up_sequence(){

    unsigned long windowStart = millis();
    while ((millis()-windowStart)<(POWER_CYCLE_DELAY/2)){delay(10);}
    changeState(WAIT_FOR_BUTTON_HOLD);
    KeypadPower.readKey(0);
    while (true){
        KeypadPower.readKey();
        int windowsize = millis() - windowStart;
        switch (sysState){
            case WAIT_FOR_BUTTON_HOLD:
                if (KeypadPower.buttonState) {//} || (windowsize > POWER_CYCLE_DELAY*3/4)) {
                    windowStart = millis() - POWER_CYCLE_DELAY/2;
                    changeState(INITIAL_BOOT);
                    Serial.println("Waiting for Release of Power Button");
                } else if (windowsize>=POWER_CYCLE_DELAY) changeState(SHUTDOWN);
                break;
            case INITIAL_BOOT:
                if (windowsize>=POWER_CYCLE_DELAY) {
                    changeState(WAIT_FOR_BUTTON_RELEASE);
                }   else if (KeypadPower.keyState==RELEASED){
                        changeState(SHUTDOWN);
                        Serial.println("First key released too early — going back to sleep");
                }
                break;
                
            case WAIT_FOR_BUTTON_RELEASE:
                if (KeypadPower.keyState==RELEASED){
                    changeState(WAIT_FOR_CONFIRMATION);
                    KeypadPower.readKey(1);
                    Serial.println("First key released — waiting for second key");
                    windowStart = millis();
                }
                break;
            case WAIT_FOR_CONFIRMATION:
                if (windowsize > POWER_CYCLE_DELAY) {
                    changeState(SHUTDOWN);
                    Serial.println("Second key pressed too late — going back to sleep");
                } else if (KeypadPower.stateChanged){
                    changeState(SETUP);
                }
                break;
            case SHUTDOWN:
                Serial.println("invalid boot sequence");
                enterDeepSleep();
                return;
                break;
            case SETUP:
                if (KeypadPower.buttonState==IDLE) return;
                break;
        }
        if (windowsize > 10000) changeState(SHUTDOWN);
        delay(10);
        
    }
}
void validate_wake_up_reason(){
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_GPIO){
        enterDeepSleep();
    }
}
void setup() {
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(gpio_num_t(power_row_GPIO));
    // validate_wake_up_reason();
    Serial.begin(9600);
    displayReady = display.init();
    timestamps.now = millis();
    changeState(INITIAL_BOOT);
    changeState(CAMERA_DISCONNECTED);
    setupKeypad(false);
    timestamps.now = millis();
    ++bootCount;
    // validate_wake_up_sequence();
    changeState(SETUP);
    KeypadPower.readKey(0);
    Serial.println("Boot number: " + String(bootCount));
    Serial.println("Starting BLEKeyboard");
    bleKeyboard.setDebugLevel(HIDLogLevel::Normal);
    bleKeyboard.setKeyGap(1);
    bleKeyboard.setTapDelay(10);
    bleKeyboard.begin();
    setupKeypad(true);
    changeState(RUNNING);
    // changeState(CAMERAS_CONNECTING);
}


void loop() {
    timestamps.now = millis();
    // bleKeyboard.setBatteryLevel(75);
    sendKeys();
    RecordingState();
    shutdown();
}  