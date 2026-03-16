#include <Arduino.h>
#include <HijelHID_BLEKeyboard.h>
#include <Keypad.h>

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

typedef enum {INITIAL_BOOT, WAIT_FOR_CONFIRMATION, SETUP, RUNNING, INITIAL_SHUTDOWN, SHUTDOWN, OFF} KeyboardState;
const String state_String[] = {"INITIAL_BOOT", "WAIT_FOR_CONFIRMATION", "SETUP", "RUNNING", "INITIAL_SHUTDOWN", "SHUTDOWN", "OFF"};
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

// Key keypad[ROWS][COLS];


const uint8_t row_GPIOs[ROWS] = {D6,D5,D4,D3};
const uint8_t col_GPIOs[COLS] = {D2,D1,D0};

const byte Power_row = 3;
const byte confirm_row = 1;
const byte Power_col = 1;

const uint8_t power_row_GPIOs[] = {row_GPIOs[Power_row],row_GPIOs[confirm_row] };
const uint8_t power_row_GPIO = row_GPIOs[Power_row];
// const uint8_t confirm_row_GPIO = row_GPIOs[confirm_row];
const uint8_t power_col_GPIO = col_GPIOs[Power_col];

// const uint16_t DEBOUNCE_MS  = 50;
// const uint16_t HOLD_TIME = 500;
// const uint16_t REPEAT_DELAY = 200;
// const uint16_t REPEAT_ACCELERATION  = 20;
// const uint16_t REPEAT_MAX_RATE  = 50;
const uint16_t POWER_CYCLE_DELAY = 3000;

HijelHID_BLEKeyboard bleKeyboard("XCTrack Keypad", "TS", 50);


unsigned long now;

bool recording = false;
unsigned long pre_shutdown_release = 0;

RTC_DATA_ATTR int bootCount = 0;

// Key PowerKey  ;// = keypad[Power_row][Power_col];
// Key ConfirmKey;// = keypad[confirm_row][Power_col];

Keypad MainKeyPad;
Keypad PowerKeyPad;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
esp_sleep_wakeup_cause_t print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    case ESP_SLEEP_WAKEUP_GPIO : Serial.println("Wakeup caused by GPIO"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
  return wakeup_reason;
}


void changeState(KeyboardState nextState){
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

void sendKeys(String KEYS){
    for (auto c:KEYS) {
        sendKey(char(c));
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

// Key setupKeypad(uint8_t *row_GPIOs, uint8_t *col_GPIOs, byte n_rows, byte n_cols, uint16_t DEBOUNCE_MS, uint16_t HOLD_TIME, uint16_t REPEAT_DELAY, uint16_t REPEAT_ACCELERATION, uint16_t REPEAT_MAX_RATE){
//     char keyMap[n_rows][n_cols];
//     char altKeyMap[n_rows][n_cols];

// }

// Key setupKeypad(char* keyMap, char* altKeyMap, uint8_t *row_GPIOs, uint8_t *col_GPIOs, byte n_rows, byte n_cols, String no_repeat, uint16_t DEBOUNCE_MS, uint16_t HOLD_TIME, uint16_t REPEAT_DELAY, uint16_t REPEAT_ACCELERATION, uint16_t REPEAT_MAX_RATE){
//     setupRows(row_GPIOs, n_rows);
//     setupCols(col_GPIOs, n_cols);
//     Key keypad[n_rows][n_cols];


// }

void SetupPowerPad(){
    PowerKeyPad.init((uint8_t*)power_row_GPIOs, power_col_GPIO, (byte)2);

}

void setupKeypad(){
    MainKeyPad.init(makeKeymap(KEYS), makeKeymap(ALT_KEYS), (uint8_t*)row_GPIOs, (uint8_t*)col_GPIOs, ROWS, COLS, no_repeat);
    // for (uint8_t r = 0; r < ROWS; r++) {
    //     pinMode(row_GPIOs[r], OUTPUT);
    //     digitalWrite(row_GPIOs[r], HIGH);
    //     for (uint8_t c = 0; c < COLS; c++){
    //         keypad[r][c].init(KEYS[r][c], ALT_KEYS[r][c], row_GPIOs[r], col_GPIOs[c], no_repeat, DEBOUNCE_MS, HOLD_TIME, REPEAT_DELAY, REPEAT_ACCELERATION, REPEAT_MAX_RATE);
    //     }
    // }
    
    // for (uint8_t c = 0; c < COLS; c++) pinMode(col_GPIOs[c], INPUT_PULLUP);
    // PowerKey = keypad[Power_row][Power_col];
    // ConfirmKey = keypad[confirm_row][Power_col];
}

void getKeys(){
    // sendKeys(MainKeyPad.getKeys());
    // for (uint8_t r = 0; r < ROWS; r++) {
    //     digitalWrite(row_GPIOs[r], LOW);
    //     delayMicroseconds(10);
    //     for (uint8_t c = 0; c < COLS; c++) {
    //         sendKey(keypad[r][c].update(digitalRead(col_GPIOs[c]) == LOW));
    //     }
    //     digitalWrite(row_GPIOs[r], HIGH);
    // }
}
void enterDeepSleep() {
    
    changeState(SHUTDOWN);
    Serial.println("Shutdown confirmed");
    Serial.println("Entering Deep Sleep"); 
    Serial.println("Ending BLEKeyboard");
  // Shut down Bluetooth cleanly first
  bleKeyboard.flush();
  bleKeyboard.end();

  // Small delay to let BT stack finish shutting down
  delay(100);

    Serial.println("prepare Pins");

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

  // Drive row 3 LOW
  for (int c=0; c<COLS; c++) pinMode(col_GPIOs[c], INPUT);

//   // Set all cols as pullup inputs
//   pinMode(4, INPUT_PULLUP);
//   pinMode(3, INPUT_PULLUP);
//   pinMode(2, INPUT_PULLUP);

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
    PowerKeyPad.setKey(0);
    // PowerKey = keypad[Power_row][Power_col];
    switch (kbdState){

        case RUNNING:
            if (PowerKeyPad.veryLongPress(POWER_CYCLE_DELAY)){
                changeState(INITIAL_SHUTDOWN);
                Serial.println("Waiting for Release of Power Button");
            }
            break;
        case INITIAL_SHUTDOWN:
            if (PowerKeyPad.keyState == RELEASED){
                pre_shutdown_release = now;
                changeState(WAIT_FOR_CONFIRMATION);
                Serial.println("Waiting for Confirmation Button");
                PowerKeyPad.setKey(1);

            }
            break;
        case WAIT_FOR_CONFIRMATION:
            // ConfirmKey = keypad[confirm_row][Power_col];
            if ((now - pre_shutdown_release) > POWER_CYCLE_DELAY){
                changeState(RUNNING);
                Serial.println("Confirmation didn't happen in time");
            } else if (PowerKeyPad.stateChanged){
                enterDeepSleep();
            }

    }
    // if (keyState[Power_row][Power_col] && ((now - pressTime[Power_row][Power_col])>POWER_CYCLE_DELAY))  pre_shutdown_release = now;
    // if (keyState[confirm_row][Power_col] && !keyState[Power_row][Power_col] && ((now - pre_shutdown_release)< POWER_CYCLE_DELAY)) enterDeepSleep();
}

bool validate_wake_up_sequence(){
    PowerKeyPad.setKey(0);
    PowerKeyPad.read();
    unsigned long hold_start = PowerKeyPad.start;
    while(!PowerKeyPad.buttonState) {
        delay(10);
        PowerKeyPad.read();
        if ((millis()-now)>(POWER_CYCLE_DELAY*3/4)) break;
    }
    while (PowerKeyPad.buttonState) {
        if ((millis()-now)>POWER_CYCLE_DELAY) break;
        delay(10);
        PowerKeyPad.read();
    }
    if (millis() - now < POWER_CYCLE_DELAY) {
        Serial.println("First key released too early — going back to sleep");
        return false;
    }
    Serial.println("Waiting for Release of Power Button");
    while (!PowerKeyPad.stateChanged){
        delay(10);
        PowerKeyPad.read();
    }

    Serial.println("First key released — waiting for second key");
    changeState(WAIT_FOR_CONFIRMATION);
    PowerKeyPad.setKey(1);
    unsigned long windowStart = millis();
    while ((millis() - windowStart) < (2 * PowerKeyPad.debounceTime)){
        
        PowerKeyPad.read();
        delay(10);
    }
    while (!PowerKeyPad.stateChanged){
        if (millis() - windowStart > POWER_CYCLE_DELAY) {
            Serial.println("Second key pressed too late — going back to sleep");
            return false;
        }
        delay(10);
        PowerKeyPad.read();
    }

    Serial.println("Success");
    return true;


}
void setup() {
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(gpio_num_t(power_row_GPIO));
    Serial.begin(9600);
    delay(1000);
    now = millis();
    // delay(2000); //Take some time to open up the Serial Monitor
    changeState(INITIAL_BOOT);
    // SetupPowerPad();
    // setupKeypad();
    while ((millis()-now)<(POWER_CYCLE_DELAY/2)){delay(10);}
    //Increment boot number and print it every reboot
    ++bootCount;
    // if (!validate_wake_up_sequence()){
    //     // delay(2000);
    //     Serial.println("invalid boot sequence");
    //     enterDeepSleep();
    // }
    changeState(SETUP);
    setupKeypad();
    Serial.println("Boot number: " + String(bootCount));

    Serial.println("Starting BLEKeyboard");

    bleKeyboard.begin();

    changeState(RUNNING);
}


void loop() {
    now = millis();
    bleKeyboard.setBatteryLevel(75);
    getKeys();
    shutdown();
}  