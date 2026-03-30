
#pragma once


// #include <Arduino.h>


// ── Host GATT services (host acts as peripheral for these) ────────────────────
#define HOST_SERVICE_UUID           "4fc05c90-6287-4618-93f8-702ba6e440f5"
#define HOST_NMEA_CHAR_UUID         "dc1ccec6-30a8-4ebc-9210-c7531e7cb4e1"


// Standard BLE Battery Service (Bluetooth SIG)
#define BATTERY_SERVICE_UUID    "180F"
#define BATTERY_CHAR_UUID       "2A19"   // "Battery Level" characteristic

// ── Custom GATT service UUID ──────────────────────────────────────────────────
#define SERVICE_UUID        "9f710bb7-e2c1-4a70-8af2-aae4226b60c4"

// Host writes a single byte here to send commands
#define CMD_CHAR_UUID       "49d0ae2e-74c0-408e-bdad-01986a24b40f"

// Client sends a single byte notify here to ACK
#define ACK_CHAR_UUID       "57a0e065-c8bf-4164-a0d0-48d4858bbbff"

// Host sends time to Cleints
#define TIME_CHAR_UUID      "fa410106-446e-45a4-b44a-e0b411d14bb1"

// Nordic UART Service — compatible with Serial Bluetooth Terminal app
#define NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone writes here
#define NUS_TX_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // host notifies here


#define CLIENT_WATCHDOG_MS 30000

// ── Command bytes ─────────────────────────────────────────────────────────────
enum class CMD : byte{
    START,
    STOP
};
const uint8_t SEND(CMD cmd); 
const uint8_t CONF(CMD cmd); 
const uint8_t ACK(CMD cmd); 

const char* toString(uint8_t msg);

#define AWAKE 0x99

// ── Tuning ────────────────────────────────────────────────────────────────────
#define DISCOVERY_TIMEOUT_MS  60000
#define ACK_TIMEOUT_MS        3000
#define MAX_CLIENTS           5


#define BAUD 9600
#define INITIAL_DELAY 2000

#define Max_TX_Power_db 9


enum class CameraState : byte {
    OFF,
    DISCONNECTED, 
    READY, 
    BOOTING, 
    RECORDING, 
    STOPPING, 
    STOPPED,
    WAIT_FOR_BOOT,
    WAIT_FOR_STOP,
};
const char* toString(CameraState state);

enum class SystemState : byte {INITIAL_BOOT,WAIT_FOR_BUTTON_HOLD, WAIT_FOR_TIME, WAIT_FOR_BUTTON_RELEASE, WAIT_FOR_CONFIRMATION, SETUP, WIFI_CONNECTING, NTP_CONNECTION, BLUETOOTH_CONNECTING, RUNNING, INITIAL_SHUTDOWN, WAIT_FOR_SHUTDOWN_CONFIRMATION, SHUTDOWN, OFF, CHARGING};
const char* toString(SystemState state);
const char* dispString(SystemState state);


#define HostMAC "98:3d:ae:ab:a4:d6"




// US18650VTC6
static const float BATT_INTERNAl_RESISTANCE_mOhm = 30.0f;
static const int BATT_TABLE_SIZE = 15;
static const int BATT_VOLTAGE_TABLE[] = {280, 288, 297, 309, 324, 339, 347, 353, 364, 387, 403, 405, 409, 412, 417};
static const int BATT_SOC_TABLE[] = {0, 1, 3, 6, 12, 21, 24, 31, 39, 67, 80, 84, 94, 97, 100};



int stateOfCharge(int V100, int &i);

class BatteryFilter {
public:
    BatteryFilter(float alpha = 0.05f) : _alpha(alpha), _initialized(false) {}

    float update(float raw) {
        if (!_initialized) {
            _value = raw;
            _initialized = true;
        } else {
            _value = _alpha * raw + (1.0f - _alpha) * _value;
        }
        return _value;
    }

    float value() const { return _value; }

private:
    float _alpha;
    float _value = 0;
    bool  _initialized;
};


class Battery {
    public:
        Battery() : _R_mOhm(BATT_INTERNAl_RESISTANCE_mOhm), V(0), V_c(0), I_mA(0), _i_soc(0), soc(0){}
        float V;
        float V_c;
        float I;
        int I_mA;
        int soc;

        void update(float V_new, float I_new){
            V = _Vfilter.update(V_new);
            I = _Ifilter.update(I_new);
            I_mA = int(I * 1000);
            V_c = V + I * float(_R_mOhm) / 1000000;
            int V100 = int(V_c * 100);
            soc = stateOfCharge(V100, _i_soc);
        }

        void update(float V){update(V, 0);}

    private:
        float _R_mOhm;
        BatteryFilter _Vfilter;
        BatteryFilter _Ifilter;
        int _i_soc;


};