// #include <Arduino.h>

#include <HijelHID_BLEKeyboard.h>

#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'}
};
String repeat_keys = "234689";
byte rowPins[ROWS] = {2,3,4,5}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {6,7,21}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


const char SPECIAL_ENTER     = '#';
const char SPECIAL_BACKSPACE = '*';

const uint16_t DEBOUNCE_MS  = 50;
const uint16_t REPEAT_DELAY = 500;
const uint16_t REPEAT_RATE  = 80;

// BleKeyboard bleKeyboard("ESP32 Keypad", "DIY", 100);
HijelHID_BLEKeyboard bleKeyboard;

bool     keyState[4][3]   = {};
bool     lastState[4][3]  = {};
uint32_t pressTime[4][3]  = {};
uint32_t lastRepeat[4][3] = {};

// put function declarations here:
// int myFunction(int, int);



unsigned long loopCount;
unsigned long startTime;
String msg;

void sendKey(char c) {
  if (!bleKeyboard.isConnected()) return;
  if (c == '#')         bleKeyboard.tap(MEDIA_VOLUME_UP);
  else if (c == '*') bleKeyboard.tap(MEDIA_VOLUME_DOWN);
  else if (c == '0') bleKeyboard.tap(MEDIA_SLEEP);
  // else if (c == '1') bleKeyboard.tap(MEDIA_BRIGHTNESS_UP);
  // else if (c == '7') bleKeyboard.tap(MEDIA_BRIGHTNESS_DOWN);
  	
  else                             bleKeyboard.write((uint8_t)c);
}

void setup() {
    Serial.begin(9600);
    Serial.println("Starting BLE work!");

// // In setup(), before bleKeyboard.begin():
// esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
// esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    bleKeyboard.begin();
    loopCount = 0;
    startTime = millis();
    msg = "";
}


void loop() {
    loopCount++;
    if ( (millis()-startTime)>5000 ) {
        Serial.print("Average loops per second = ");
        Serial.println(loopCount/5);
        startTime = millis();
        loopCount = 0;
    }

    // Fills kpd.key[ ] array with up-to 10 active keys.
    // Returns true if there are ANY active keys.
    if (kpd.getKeys())
    {
        for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
        {
            if ( kpd.key[i].stateChanged )   // Only find keys that have changed state.
            {
                switch (kpd.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
                    case PRESSED:
                    msg = " PRESSED.";
                break;
                    case HOLD:
                    msg = " HOLD for ";
                    msg += String(kpd.key[i].wait);
                    if (repeat_keys.indexOf(kpd.key[i].kchar)>-1){
                        sendKey(kpd.key[i].kchar);
                    }
                break;
                    case RELEASED:
                    msg = " RELEASED.";
                    sendKey(kpd.key[i].kchar);
                break;
                    case IDLE:
                    msg = " IDLE.";
                }
                Serial.print("Key ");
                Serial.print(kpd.key[i].kchar);
                Serial.println(msg);
            }
//             if (kpd.key[i].kstate == HOLD){
//                 msg = " HOLD for ";
//                 msg += String(millis() - kpd.key[i].start);
//                 Serial.print("Key ");
//                 Serial.print(kpd.key[i].kchar);
//                 Serial.println(msg);
// }
        }
    }
}  // End loop