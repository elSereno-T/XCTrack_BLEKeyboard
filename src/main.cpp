#include <Arduino.h>
#include <HijelHID_BLEKeyboard.h>

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

typedef enum {
    INITIAL_BOOT,
    WAIT_FOR_BOOT_CONFIRMATION,
    SETUP,
    RUNNING,
    INITIAL_SHUTDOWN,
    SHUTDOWN,
}keyboard_state;

const String key_chars =  String(ESC) + String(Backspace) + String(Power) + String(Enter) + String(Tab) ;
const uint8_t key_array[] = {KEY_ESCAPE,   KEY_BACKSPACE,      KEY_POWER,      KEY_RETURN,     KEY_TAB};
const String media_chars =     String(Vol_p) +  String(Vol_m) +    String(Home) +      String(BL);
const uint16_t media_array[] = {MEDIA_VOLUME_UP, MEDIA_VOLUME_DOWN, MEDIA_BROWSER_HOME, MEDIA_DISPLAY_BACKLIGHT};
const String no_repeat = String(Power) + String(AltTab) + String(Enter)+ String(Home) + String(REC);

const byte ROWS = 4;
const byte COLS = 3;

const char KEYS[ROWS][COLS] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {Vol_m,'0',Vol_p}
};

const char HOLD_KEYS[ROWS][COLS] = {
    {ESC,'2',Home},
    {'4',XCT,'6'},
    {REC,'8',AltTab},
    {Backspace,Power,Enter}
};

const uint8_t row_GPIOs[ROWS] = {D6,D5,D4,D3};
const uint8_t col_GPIOs[COLS] = {D2,D1,D0};
const int a = SCL;

const byte Power_row = 3;
const byte confirm_row = 1;
const byte Power_col = 1;

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

bool     keyState    [ROWS][COLS] = {};
bool     lastState   [ROWS][COLS] = {};
bool     repeatButton[ROWS][COLS] = {};
uint32_t pressTime   [ROWS][COLS] = {};
uint32_t changeTime  [ROWS][COLS] = {};
uint32_t lastRepeat  [ROWS][COLS] = {};
uint16_t waitTime    [ROWS][COLS] = {};

unsigned long now;

bool recording = false;
unsigned long pre_shutdown_release = 0;

RTC_DATA_ATTR int bootCount = 0;

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

    if (KEY == XCT) XCTrack();
    else if (KEY == AltTab) ALT_TAB();
    else if (KEY == REC) Toggle_Recording();
    else {
        Serial.println(KEY);
        if (key_chars.indexOf(String(KEY))>-1)bleKeyboard.tap(key_array[key_chars.indexOf(String(KEY))]);
        else if (media_chars.indexOf(String(KEY))>-1)bleKeyboard.tap(media_array[media_chars.indexOf(String(KEY))]);
        else bleKeyboard.write((uint8_t)KEY);
    }
}


void setupKeypad(){
    for (uint8_t r = 0; r < ROWS; r++) {
        pinMode(row_GPIOs[r], OUTPUT);
        digitalWrite(row_GPIOs[r], HIGH);
        for (uint8_t c = 0; c < COLS; c++){
            repeatButton[r][c] = (KEYS[r][c] == HOLD_KEYS[r][c]) ;
            changeTime[r][c] = 0;
            keyState[r][c] = false;
            lastState[r][c] = false;
            if (repeatButton[r][c]) waitTime[r][c] = REPEAT_DELAY;
            else waitTime[r][c] = HOLD_TIME;
        }
    }
    
    for (uint8_t c = 0; c < COLS; c++) pinMode(col_GPIOs[c], INPUT_PULLUP);
}

void getKeys(){

    for (uint8_t r = 0; r < ROWS; r++) {
        digitalWrite(row_GPIOs[r], LOW);
        delayMicroseconds(10);
        for (uint8_t c = 0; c < COLS; c++) {
            bool pressed = (digitalRead(col_GPIOs[c]) == LOW);
            if (pressed != lastState[r][c]) {
                delay(DEBOUNCE_MS);
                pressed = (digitalRead(col_GPIOs[c]) == LOW);
                if (pressed != lastState[r][c])
                lastState[r][c] = pressed;
            }   
                
            if (pressed && !keyState[r][c]) {
                keyState[r][c] = true;
                pressTime[r][c] = lastRepeat[r][c] = now;
            } else if (!pressed && keyState[r][c]) {
                keyState[r][c] = false;
                if (now - pressTime[r][c] < min(HOLD_TIME, waitTime[r][c])) sendKey(KEYS[r][c]);
                if (repeatButton[r][c]) waitTime[r][c] = REPEAT_DELAY;
                else waitTime[r][c] = HOLD_TIME;
            } else if (pressed && keyState[r][c]) {
                if ((now - lastRepeat[r][c]) >= waitTime[r][c]){
                    lastRepeat[r][c] += waitTime[r][c];
                    sendKey(HOLD_KEYS[r][c]);
                    if (repeatButton[r][c]){
                        waitTime[r][c] = max(int(REPEAT_MAX_RATE), int(waitTime[r][c] - REPEAT_ACCELERATION));
                    }
                    else if (no_repeat.indexOf(HOLD_KEYS[r][c])>-1) waitTime[r][c] = 20000;
                    else {
                        waitTime[r][c] = HOLD_TIME * 2;
                    }
                }
            }
        }
        
        digitalWrite(row_GPIOs[r], HIGH);
    }
}
void enterDeepSleep() {
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
    if (keyState[Power_row][Power_col] && ((now - pressTime[Power_row][Power_col])>POWER_CYCLE_DELAY))  pre_shutdown_release = now;
    if (keyState[confirm_row][Power_col] && !keyState[Power_row][Power_col] && ((now - pre_shutdown_release)< POWER_CYCLE_DELAY)) enterDeepSleep();
}

bool validate_wake_up_sequence(){

    //Print the wakeup reason for ESP32
    // if (print_wakeup_reason()==0 && bootCount == 1) return true;
    delayMicroseconds(10);
    pinMode(power_col_GPIO, OUTPUT);
    delayMicroseconds(10);
    pinMode(power_row_GPIO, INPUT_PULLUP);
    pinMode(confirm_row_GPIO, INPUT_PULLUP);
    delayMicroseconds(10);
    digitalWrite(power_col_GPIO, LOW);
    delayMicroseconds(10);
    unsigned long hold_start = millis();
    while (digitalRead(power_row_GPIO) == LOW) {
        // if (millis() - hold_start > POWER_OFF_TIME) break;
        delay(10);
    }
    if (millis() - hold_start < POWER_CYCLE_DELAY) {
        Serial.println("First key released too early — going back to sleep");
        return false;
    }

     Serial.println("First key released — waiting for second key");

    unsigned long windowStart = millis();
    while (digitalRead(confirm_row_GPIO) == HIGH){
        if (millis() - windowStart > POWER_CYCLE_DELAY) {
            Serial.println("Second key pressed too late — going back to sleep");
            return false;
        }
        delay(10);
    }

    Serial.println("Success");
    return true;


}
void setup() {
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(gpio_num_t(power_row_GPIO));
    now = 0;
    Serial.begin(9600);
    //Increment boot number and print it every reboot
    ++bootCount;
    if (!validate_wake_up_sequence()){
        delay(2000);
        Serial.println("invalid boot sequence");
        enterDeepSleep();
    }
    delay(2000); //Take some time to open up the Serial Monitor

    Serial.println("Boot number: " + String(bootCount));

    Serial.println("Starting BLEKeyboard");

    bleKeyboard.begin();

    setupKeypad();

}


void loop() {
    now = millis();
    bleKeyboard.setBatteryLevel(75);
    getKeys();
    shutdown();
}  