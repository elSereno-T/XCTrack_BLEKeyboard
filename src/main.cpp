#include <Arduino.h>

#include <HijelHID_BLEKeyboard.h>

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns

// uint16_t KEYS[ROWS][COLS] = {
//     {KEY_1, KEY_2, KEY_3},
//     {KEY_4, KEY_5, KEY_6},
//     {KEY_7, KEY_8, KEY_9},
//     {MEDIA_VOLUME_DOWN, KEY_0, MEDIA_VOLUME_UP},
// };
// uint16_t HOLD_KEYS[ROWS][COLS] = {
//     {KEY_ESCAPE, KEY_2, MEDIA_BROWSER_HOME},
//     {KEY_4, KEY_5, KEY_6},
//     {KEY_7, KEY_8, KEY_9},
//     {MEDIA_VOLUME_DOWN, KEY_0, MEDIA_VOLUME_UP},

// };
char keys[ROWS][COLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'}
};
char hold_keys[ROWS][COLS] = {
{'a','2','c'},
{'4','e','6'},
{'g','8','i'},
{'j','k','l'}
};
String repeat_keys = "2468";
byte rowPins[ROWS] = {2,3,4,5};
byte colPins[COLS] = {6,7,21};



const uint16_t DEBOUNCE_MS  = 50;
const uint16_t HOLD_TIME = 500;
const uint16_t REPEAT_DELAY = 200;
const uint16_t REPEAT_ACCELERATION  = 20;
const uint16_t REPEAT_MAX_RATE  = 100;

HijelHID_BLEKeyboard bleKeyboard("XCTrack Keypad", "TS", 50);

bool     keyState[ROWS][COLS]   = {};
bool     lastState[ROWS][COLS]  = {};
uint32_t pressTime[ROWS][COLS]  = {};
uint32_t changeTime[ROWS][COLS] = {};
uint32_t lastRepeat[ROWS][COLS] = {};
bool     repeatButton[ROWS][COLS] = {};
uint16_t waitTime[ROWS][COLS] = {};



unsigned long loopCount;
unsigned long startTime;
unsigned long now;


void sendKey(char c) {
    Serial.printf("Key: %c\n", c);
    if (!bleKeyboard.isConnected()) return;
    switch (c)
    {
    case '#':
        bleKeyboard.tap(MEDIA_VOLUME_UP);
        break;
    case '*':
        bleKeyboard.tap(MEDIA_VOLUME_DOWN);
        break;
    case 'k':
        // bleKeyboard.tap(MEDIA_SLEEP);
        bleKeyboard.tap(KEY_POWER);
        break;
    case 'l':
        bleKeyboard.tap(KEY_RETURN);
        break;
    case 'j':
        bleKeyboard.tap(KEY_BACKSPACE);
        break;
    case 'c':
        bleKeyboard.tap(MEDIA_BROWSER_HOME);
        break;
    case 'a':
        bleKeyboard.tap(KEY_ESCAPE);
    //     // bleKeyboard.tap(KEY_APPLICATION);
    //     // bleKeyboard.tap(MEDIA_TASK_MANAGER);
    //     bleKeyboard.tap(MEDIA_BROWSER_FORWARD);
        break;
    case 'e':
        bleKeyboard.print("XCTRACK");
        break;
    default:
        bleKeyboard.write((uint8_t)c);
        break;
    }                        
}

void setupKeypad(){
      for (uint8_t r = 0; r < ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  for (uint8_t c = 0; c < COLS; c++){
     repeatButton[r][c] = (repeat_keys.indexOf(keys[r][c]) > -1);
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
        if (now - pressTime[r][c] < min(HOLD_TIME, waitTime[r][c])) sendKey(keys[r][c]);
        if (repeatButton[r][c]) waitTime[r][c] = REPEAT_DELAY;
        else waitTime[r][c] = HOLD_TIME;
      } else if (pressed && keyState[r][c]) {
        if ((now - lastRepeat[r][c]) >= waitTime[r][c]){
            lastRepeat[r][c] += waitTime[r][c];
            if (repeatButton[r][c]){
                sendKey(keys[r][c]);
                waitTime[r][c] = max(int(REPEAT_MAX_RATE), int(waitTime[r][c] - REPEAT_ACCELERATION));
            }
            else {
                sendKey(hold_keys[r][c]);
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
    loopCount = 0;
    startTime = 0;

    setupKeypad();
}


void loop() {
    now = millis();
    loopCount++;
    if ( (now-startTime)>5000 ) {
        Serial.print("Average loops per second = ");
        Serial.println(loopCount/5);
        startTime +=5000;
        loopCount = 0;
    }
    getKeys();
}  