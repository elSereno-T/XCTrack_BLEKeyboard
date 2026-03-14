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
byte rowPins[ROWS] = {2,3,4,5};
byte colPins[COLS] = {6,7,21};

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
  for (uint8_t c = 0; c < COLS; c++)
    pinMode(colPins[c], INPUT_PULLUP);
}

void getKeys(){

  for (uint8_t r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(10);
    for (uint8_t c = 0; c < COLS; c++) {
        // if (changeTime[r][c] - now < DEBOUNCE_MS){continue;}
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
        // Serial.printf("Key: %c\n", keys[r][c]);
        // sendKey(keys[r][c]);
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
                // sendKey(KEYS[r][c]);
                waitTime[r][c] = max(int(REPEAT_MAX_RATE), int(waitTime[r][c] - REPEAT_ACCELERATION));
            }
            else {
                waitTime[r][c] = HOLD_TIME * 2;
            }
        }
        // if ((now - pressTime[r][c]) > REPEAT_DELAY &&
        //     (now - lastRepeat[r][c]) > REPEAT_RATE) {
        //   lastRepeat[r][c] = now;
        //   sendKey(keys[r][c]);
        }
      }
    
    digitalWrite(rowPins[r], HIGH);
  }
}

void setup() {
    now = 0;
    Serial.begin(9600);
    Serial.println("Starting BLE work!");

    bleKeyboard.begin();

    setupKeypad();

}


void loop() {
    now = millis();
    getKeys();
}  