#include <Arduino.h>
#include <HijelHID_BLEKeyboard.h>
#include <Key.h>

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

typedef enum {INITIAL_BOOT,WAIT_FOR_BUTTON_HOLD, WAIT_FOR_BUTTON_RELEASE, WAIT_FOR_CONFIRMATION, SETUP, RUNNING, INITIAL_SHUTDOWN, SHUTDOWN, OFF} KeyboardState;
const String state_String[] = {"INITIAL_BOOT", "WAIT_FOR_BUTTON_HOLD", "WAIT_FOR_BUTTON_RELEASE", "WAIT_FOR_CONFIRMATION", "SETUP", "RUNNING", "INITIAL_SHUTDOWN", "SHUTDOWN", "OFF"};
KeyboardState kbdState = OFF;

const String key_chars =  String(ESC) + String(Backspace) + String(Power) + String(Enter) + String(Tab) ;
const uint8_t key_array[] = {KEY_ESCAPE,   KEY_BACKSPACE,      KEY_POWER,      KEY_RETURN,     KEY_TAB};
const String media_chars =     String(Vol_p) +  String(Vol_m) +    String(Home) +      String(BL);
const uint16_t media_array[] = {MEDIA_VOLUME_UP, MEDIA_VOLUME_DOWN, MEDIA_BROWSER_HOME, MEDIA_DISPLAY_BACKLIGHT};
const String no_repeat = String(Power) + String(AltTab) + String(Enter)+ String(Home) + String(REC) + String(ESC) + String(XCT);

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

Key keypad[ROWS][COLS];


const uint8_t row_GPIOs[ROWS] = {D6,D5,D4,D3};
const uint8_t col_GPIOs[COLS] = {D2,D1,D0};

const byte Power_row = 3;
const byte confirm_row = 1;
const byte Power_col = 1;

const uint8_t power_row_GPIOs[] = {row_GPIOs[Power_row],row_GPIOs[confirm_row] };
const uint8_t power_row_GPIO = row_GPIOs[Power_row];
const uint8_t confirm_row_GPIO = row_GPIOs[confirm_row];
const uint8_t power_col_GPIO = col_GPIOs[Power_col];

const uint16_t DEBOUNCE_MS  = 50;
const uint16_t HOLD_TIME = 500;
const uint16_t REPEAT_DELAY = 200;
const uint16_t REPEAT_ACCELERATION  = 20;
const uint16_t REPEAT_MAX_RATE  = 50;
const uint16_t POWER_CYCLE_DELAY = 3000;

HijelHID_BLEKeyboard bleKeyboard("XCTrack Keypad", "TS", 50);


unsigned long now;

bool recording = false;
unsigned long pre_shutdown_release = 0;

RTC_DATA_ATTR int bootCount = 0;

Key PowerKey  ;
Key ConfirmKey;



void changeState(KeyboardState nextState){
    if (kbdState == nextState) return;
    Serial.println("Going from "+state_String[kbdState] + " to " + state_String[nextState]);
    kbdState = nextState;
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

void setupRows(uint8_t *GPIOs, byte n_rows){
    for (byte ii=0; ii<n_rows; ii++){
        pinMode(GPIOs[ii], OUTPUT);
        digitalWrite(GPIOs[ii], HIGH);
    }

}
void setupCols(uint8_t *GPIOs, byte n_cols){
    for (byte ii=0; ii<n_cols; ii++){
        pinMode(GPIOs[ii], INPUT_PULLUP);
    }
}


void setupKeypad(){
    for (uint8_t r = 0; r < ROWS; r++) {
        pinMode(row_GPIOs[r], OUTPUT);
        digitalWrite(row_GPIOs[r], HIGH);
        for (uint8_t c = 0; c < COLS; c++){
            keypad[r][c].init(KEYS[r][c], ALT_KEYS[r][c], row_GPIOs[r], col_GPIOs[c], no_repeat, DEBOUNCE_MS, HOLD_TIME, REPEAT_DELAY, REPEAT_ACCELERATION, REPEAT_MAX_RATE);
        }
    }
    
    for (uint8_t c = 0; c < COLS; c++) pinMode(col_GPIOs[c], INPUT_PULLUP);
    PowerKey = keypad[Power_row][Power_col];
    ConfirmKey = keypad[confirm_row][Power_col];
}

void getKeys(){
    for (uint8_t r = 0; r < ROWS; r++) {
        digitalWrite(row_GPIOs[r], LOW);
        delayMicroseconds(10);
        for (uint8_t c = 0; c < COLS; c++) {
            sendKey(keypad[r][c].read());
        }
        digitalWrite(row_GPIOs[r], HIGH);
    }
}
void enterDeepSleep() {
    
    changeState(SHUTDOWN);
    Serial.println("Shutdown confirmed");
    Serial.println("Entering Deep Sleep"); 
    Serial.println("Ending BLEKeyboard");
    // Shut down Bluetooth cleanly first
    bleKeyboard.end();

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
    PowerKey = keypad[Power_row][Power_col];
    switch (kbdState){

        case RUNNING:
            if (PowerKey.veryLongPress(POWER_CYCLE_DELAY)){
                changeState(INITIAL_SHUTDOWN);
                Serial.println("Waiting for Release of Power Button");
            }
            break;
        case INITIAL_SHUTDOWN:
            if (PowerKey.keyState == RELEASED){
                pre_shutdown_release = now;
                changeState(WAIT_FOR_CONFIRMATION);
                Serial.println("Waiting for Confirmation Button");
            }
            break;
        case WAIT_FOR_CONFIRMATION:
            ConfirmKey = keypad[confirm_row][Power_col];
            if ((now - pre_shutdown_release) > POWER_CYCLE_DELAY){
                changeState(RUNNING);
                Serial.println("Confirmation didn't happen in time");
            } else if (ConfirmKey.stateChanged){
                enterDeepSleep();
            }

    }
}

void validate_wake_up_sequence(){

    unsigned long windowStart = millis();
    while ((millis()-windowStart)<(POWER_CYCLE_DELAY/2)){delay(10);}
    changeState(WAIT_FOR_BUTTON_HOLD);
    while (true){
        PowerKey.read(true);
        ConfirmKey.read(true);
        int windowsize = millis() - windowStart;
        switch (kbdState){
            case WAIT_FOR_BUTTON_HOLD:
                if (PowerKey.buttonState) {//} || (windowsize > POWER_CYCLE_DELAY*3/4)) {
                    windowStart = millis() - POWER_CYCLE_DELAY/2;
                    changeState(WAIT_FOR_BUTTON_RELEASE);
                    Serial.println("Waiting for Release of Power Button");
                } else if (windowsize>=POWER_CYCLE_DELAY) changeState(SHUTDOWN);
                break;
            case WAIT_FOR_BUTTON_RELEASE:
                if (PowerKey.keyState==RELEASED){
                    if (windowsize>=POWER_CYCLE_DELAY) {
                        changeState(WAIT_FOR_CONFIRMATION);
                        Serial.println("First key released — waiting for second key");
                        windowStart = millis();
                    }
                    else {
                        changeState(SHUTDOWN);
                        Serial.println("First key released too early — going back to sleep");
                    }
                }
                break;
            case WAIT_FOR_CONFIRMATION:
                if (windowsize > POWER_CYCLE_DELAY) {
                    changeState(SHUTDOWN);
                    Serial.println("Second key pressed too late — going back to sleep");
                } else if (ConfirmKey.stateChanged){
                    changeState(SETUP);
                }
                break;
            case SHUTDOWN:
                Serial.println("invalid boot sequence");
                enterDeepSleep();
                return;
                break;
            case SETUP:
                if (ConfirmKey.buttonState==IDLE) return;
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
    Serial.begin(9600);
    validate_wake_up_reason();
    changeState(INITIAL_BOOT);
    setupKeypad();
    now = millis();
    ++bootCount;
    validate_wake_up_sequence();
    changeState(SETUP);
    Serial.println("Boot number: " + String(bootCount));

    Serial.println("Starting BLEKeyboard");
     bleKeyboard.setDebugLevel(HIDLogLevel::Normal);
   bleKeyboard.begin();
    changeState(RUNNING);
}


void loop() {
    now = millis();
    // bleKeyboard.setBatteryLevel(75);
    getKeys();
    shutdown();
}  