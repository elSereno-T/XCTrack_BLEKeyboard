#include <Arduino.h> 
#include <shared.h>

int stateOfCharge(int V100, int &i){
    

    if (V100<=BATT_VOLTAGE_TABLE[0]) {
        i = 0;
        return 0;
    }
    if (V100 >= BATT_VOLTAGE_TABLE[BATT_TABLE_SIZE-1]){
        i = BATT_TABLE_SIZE-2;
        return 100;
    }
    while (V100 > BATT_VOLTAGE_TABLE[i+1]){i++;}
    while (V100 < BATT_VOLTAGE_TABLE[i]){i--;}

    int v0 = BATT_VOLTAGE_TABLE[i];
    int v1 = BATT_VOLTAGE_TABLE[i+1];
    int s0 = BATT_SOC_TABLE[i];
    int s1 = BATT_SOC_TABLE[i+1];
    return s0 + (V100 - v0) * (s1 - s0) / (v1 - v0);

}


const uint8_t SEND(CMD cmd){
    switch (cmd){
        case CMD::START: return 0x01;
        case CMD::STOP:  return 0x02;
        case CMD::REPORT:  return 0x03;
        default: return 0x00;
    }
}; 
// const uint8_t CONF(CMD cmd){
//     switch (cmd){
//         case CMD::START: return 0x11;
//         case CMD::STOP:  return 0x12;
//         default: return 0x10;
//     }
// };  
// const uint8_t ACK(CMD cmd){
//     switch (cmd){
//         case CMD::START: return 0x21;
//         case CMD::STOP:  return 0x22;
//         default: return 0x20;
//     }
// }; 
const char* toString(CMD cmd){
    switch (cmd)
    {
        case CMD::START: return "START";
        case CMD::STOP: return "STOP";
        case CMD::REPORT: return "REPORT";
        default:          return "UNKNOWN";
    }
}
const char* toString(uint8_t msg){
    switch (msg)
    {
        case 0x01: return "CMD::START";
        case 0x02: return "CMD::STOP";
        case 0x03: return "CMD::REPORT";
        case 0x11: return "CONF::START";
        case 0x12: return "CONF::STOP";
        case 0x21: return "ACK::START";
        case 0x22: return "ACK::STOP";
        case 0x99: return "AWAKE";
        default: return "UNKNOWN";        
    }
}


const char* toString(CameraState state) {
    switch (state) {
        case CameraState::OFF: return "OFF";
        case CameraState::DISCONNECTED: return "DISCONNECTED";
        case CameraState::CONNECTED: return "CONNECTED";
        case CameraState::READY: return "READY";
        case CameraState::BOOTING: return "BOOTING";
        case CameraState::RECORDING: return "RECORDING";
        case CameraState::STOPPING: return "STOPPING";
        case CameraState::STOPPED: return "STOPPED";
        case CameraState::WAIT_FOR_BOOT: return "WAIT_FOR_BOOT";
        case CameraState::WAIT_FOR_STOP: return "WAIT_FOR_STOP";
        default: return "UNKNOWN";
    }
}

const char* toString(SystemState state) {
    switch (state) {
        case SystemState::INITIAL_BOOT: return "INITIAL BOOT";
        case SystemState::WAIT_FOR_BUTTON_HOLD: return "WAIT FOR BUTTON HOLD";
        case SystemState::WAIT_FOR_TIME: return "WAIT FOR TIME";
        case SystemState::WAIT_FOR_BUTTON_RELEASE: return "WAIT FOR BUTTON RELEASE";
        case SystemState::WAIT_FOR_CONFIRMATION: return "WAIT FOR CONFIRMATION";
        case SystemState::SETUP: return "SETUP";
        case SystemState::WIFI_CONNECTING: return "WIFI_CONNECTING";
        case SystemState::NTP_CONNECTION: return "NTP_CONNECTION";
        case SystemState::BLUETOOTH_CONNECTING: return "BLUETOOTH_CONNECTING";
        case SystemState::RUNNING: return "RUNNING";
        case SystemState::INITIAL_SHUTDOWN: return "INITIAL SHUTDOWN";
        case SystemState::WAIT_FOR_SHUTDOWN_CONFIRMATION: return "WAIT FOR SHUTDOWN CONFIRMATION";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
        case SystemState::OFF: return "OFF";
        case SystemState::CHARGING: return "CHARGING";
        default: return "UNKNOWN";
    }
}

const char* dispString(SystemState state) {
    switch (state) {
        case SystemState::INITIAL_BOOT: return "HOLD";
        case SystemState::WAIT_FOR_BUTTON_HOLD: return "HOLD";
        case SystemState::WAIT_FOR_TIME: return "HOLD";
        case SystemState::WAIT_FOR_BUTTON_RELEASE: return "RELEASE";
        case SystemState::WAIT_FOR_CONFIRMATION: return "CONFIRM";
        case SystemState::SETUP: return "SETUP";
        case SystemState::WIFI_CONNECTING: return "WIFI";
        case SystemState::NTP_CONNECTION: return "NTP";
        case SystemState::BLUETOOTH_CONNECTING: return "BLUETOOTH";
        case SystemState::RUNNING: return "RUNNING";
        case SystemState::INITIAL_SHUTDOWN: return "RELEASE";
        case SystemState::WAIT_FOR_SHUTDOWN_CONFIRMATION: return "CONFIRM";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
        case SystemState::OFF: return "OFF";
        default: return "UNKNOWN";
    }
}