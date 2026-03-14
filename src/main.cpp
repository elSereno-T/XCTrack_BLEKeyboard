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

String key_chars =  String(ESC) + String(Backspace) + String(Power) + String(Enter) + String(Tab) ;
uint8_t key_array[] = {KEY_ESCAPE,   KEY_BACKSPACE,      KEY_POWER,      KEY_RETURN,     KEY_TAB};
String media_chars =     String(Vol_p) +  String(Vol_m) +    String(Home) +      String(BL);
uint16_t media_array[] = {MEDIA_VOLUME_UP, MEDIA_VOLUME_DOWN, MEDIA_BROWSER_HOME, MEDIA_DISPLAY_BACKLIGHT};

const byte ROWS = 4;
const byte COLS = 3;
#define COL_PIN_MASK ((1 << D0) + (1 << D1) + (1 << D2))
char KEYS[ROWS][COLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{Vol_m,'0',Vol_p}
};
char HOLD_KEYS[ROWS][COLS] = {
{ESC,'2',Home},
{'4',XCT,'6'},
{REC,'8',AltTab},
{Backspace,Power,Enter}
};
byte rowPins[ROWS] = {D6,D5,D4,D3};
byte colPins[COLS] = {D2,D1,D0};

byte Power_row = D6;
byte Power_col = D0;

const uint16_t DEBOUNCE_MS  = 50;
const uint16_t HOLD_TIME = 500;
const uint16_t REPEAT_DELAY = 200;
const uint16_t REPEAT_ACCELERATION  = 20;
const uint16_t REPEAT_MAX_RATE  = 50;

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

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
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
        pinMode(rowPins[r], OUTPUT);
        digitalWrite(rowPins[r], HIGH);
        for (uint8_t c = 0; c < COLS; c++){
            repeatButton[r][c] = (KEYS[r][c] == HOLD_KEYS[r][c]) ;
            changeTime[r][c] = 0;
            keyState[r][c] = false;
            lastState[r][c] = false;
            if (repeatButton[r][c]) waitTime[r][c] = REPEAT_DELAY;
            else waitTime[r][c] = HOLD_TIME;
        }
    }
    
    gpio_hold_dis(gpio_num_t(Power_row));
    for (uint8_t c = 0; c < COLS; c++) pinMode(colPins[c], INPUT_PULLUP);
}

void getKeys(){

    for (uint8_t r = 0; r < ROWS; r++) {
        digitalWrite(rowPins[r], LOW);
        delayMicroseconds(10);
        for (uint8_t c = 0; c < COLS; c++) {
            bool pressed = (digitalRead(colPins[c]) == LOW);
            if (pressed != lastState[r][c]) {
                delay(DEBOUNCE_MS);
                pressed = (digitalRead(colPins[c]) == LOW);
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
                    else {
                        waitTime[r][c] = HOLD_TIME * 2;
                    }
                }
            }
        }
        
        digitalWrite(rowPins[r], HIGH);
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
    byte row = rowPins[r];
    if (row == Power_row){
        pinMode(row, OUTPUT);
        digitalWrite(row, LOW);
        gpio_hold_en(gpio_num_t(row));
    } else {

    pinMode(rowPins[r], INPUT);
    }
  }

  // Drive row 3 LOW
  pinMode(rowPins[0], OUTPUT);
  digitalWrite(rowPins[0], LOW);
  for (int c=0; c<COLS; c++) pinMode(colPins[c], INPUT);

//   // Set all cols as pullup inputs
//   pinMode(4, INPUT_PULLUP);
//   pinMode(3, INPUT_PULLUP);
//   pinMode(2, INPUT_PULLUP);

  esp_deep_sleep_enable_gpio_wakeup(BIT(D0), ESP_GPIO_WAKEUP_GPIO_LOW);

  Serial.println("Serial Flush");
  Serial.flush();
  delay(100);
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
}
void setup() {
    now = 0;
    Serial.begin(9600);
    delay(2000); //Take some time to open up the Serial Monitor

    //Increment boot number and print it every reboot
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));

    //Print the wakeup reason for ESP32
    print_wakeup_reason();
    Serial.println("Starting BLEKeyboard");

    bleKeyboard.begin();

    setupKeypad();

}


void loop() {
    now = millis();
    getKeys();
    if (now>10000) enterDeepSleep();
}  