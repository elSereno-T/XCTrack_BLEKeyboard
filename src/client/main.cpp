#include <Arduino.h>
#include <NimBLEDevice.h>
#include <shared.h>
#include <time.h>

#include <HardwareSerial.h>
#include <cstring>
#include <cstdio>

// At top of file, alongside your other state vars:
Battery sysBatt;

uint8_t battPctPrev;
static NimBLECharacteristic* pBattChar  = nullptr;   // set during setup()
#define BATT_UPDATE_INTERVAL_MS         10000         // every 5 seconds

// ── UART config ───────────────────────────────────────────────────────────────
#define RUNCAM_SERIAL   Serial1
#define RUNCAM_BAUD     115200
#define RUNCAM_RX_PIN   RX
#define RUNCAM_TX_PIN   TX

// ── Protocol constants ────────────────────────────────────────────────────────
static const uint8_t RC_HEADER           = 0xCC;
static const uint8_t RC_ERROR_HEADER     = 0x55;
static const uint8_t CMD_GET_DEVICE_INFO = 0x00;
static const uint8_t CMD_CAMERA_CONTROL  = 0x01;

// Action IDs for CMD_CAMERA_CONTROL
static const uint8_t ACTION_POWER_BTN    = 0x01;  // toggles start/stop recording
static const uint8_t ACTION_CHANGE_MODE  = 0x02;  // cycles video ↔ QR/settings

// Feature flags from GET_DEVICE_INFO (little-endian uint16)
static const uint16_t FEAT_POWER_BTN     = (1 << 0);
static const uint16_t FEAT_WIFI_BTN      = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE   = (1 << 2);
static const uint16_t FEAT_5KEY_OSD      = (1 << 3);
static const uint16_t FEAT_SETTINGS      = (1 << 4);  // advertised but NOT implemented
static const uint16_t FEAT_DISPLAYPORT   = (1 << 5);  // advertised but NOT implemented
static const uint16_t FEAT_START_REC     = (1 << 6);  // advertised but use POWER_BTN instead
static const uint16_t FEAT_STOP_REC      = (1 << 7);

// ── Timing ────────────────────────────────────────────────────────────────────
static const uint32_t BTN_PRESS_DELAY_MS  = 300;
static const uint32_t MODE_CHANGE_DELAY   = 600;
static const uint32_t RX_TIMEOUT_MS       = 300;
static const uint32_t RX_INTER_BYTE_MS    = 30;
static const uint32_t INIT_RETRY_DELAY_MS = 1000;
static const uint8_t  INIT_MAX_RETRIES    = 10;

// ── State ─────────────────────────────────────────────────────────────────────
static bool     g_cameraReady    = false;
static bool     g_isRecording    = false;
static uint16_t g_cameraFeatures = 0;
static uint8_t  g_protocolVer    = 0;
// ── CRC-8 poly 0xD5 ──────────────────────────────────────────────────────────
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
        }
    }
    return crc;
}

void drainRx() {
    uint32_t deadline = millis() + 20;
    while (millis() < deadline) {
        if (RUNCAM_SERIAL.available()) {
            RUNCAM_SERIAL.read();
            deadline = millis() + 5;
        }
    }
}

void sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen) {
    uint8_t buf[32];
    size_t totalLen = 2 + payloadLen;
    if (totalLen + 1 > sizeof(buf)) { Serial.println("[ERR] Packet too large"); return; }
    buf[0] = RC_HEADER;
    buf[1] = cmd;
    for (size_t i = 0; i < payloadLen; i++) buf[2 + i] = payload[i];
    buf[totalLen] = crc8(buf, totalLen);
    drainRx();
    RUNCAM_SERIAL.write(buf, totalLen + 1);
    RUNCAM_SERIAL.flush();
}

size_t receivePacket(uint8_t *buf, size_t maxLen) {
    size_t count = 0;
    uint32_t deadline = millis() + RX_TIMEOUT_MS;
    while (millis() < deadline && count < maxLen) {
        if (RUNCAM_SERIAL.available()) {
            buf[count++] = (uint8_t)RUNCAM_SERIAL.read();
            deadline = millis() + RX_INTER_BYTE_MS;
        }
    }
    return count;
}

bool isValidResponse(const uint8_t *buf, size_t len, size_t expectedLen) {
    if (len < expectedLen)    return false;
    if (buf[0] != RC_HEADER)  return false;
    return buf[len - 1] == crc8(buf, len - 1);
}

// ── GET_DEVICE_INFO (0x00) ────────────────────────────────────────────────────
bool getDeviceInfo() {
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);
    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (!isValidResponse(resp, n, 5)) return false;
    g_protocolVer    = resp[1];
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    return true;
}

// ── Recovery ──────────────────────────────────────────────────────────────────
bool recoverCamera() {
    for (uint8_t i = 0; i < 8; i++) {
        delay(1000);
        if (getDeviceInfo()) {
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
    }
    g_cameraReady = false;
    return false;
}

// ── Camera control ────────────────────────────────────────────────────────────
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}
void toggleRecording() {
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = !g_isRecording;
}

void startRecording(){
    if (g_isRecording) return;
    toggleRecording();
}

void stopRecording(){
    if (!g_isRecording) return;
    toggleRecording();
}

void changeMode() {
    sendCameraControl(ACTION_CHANGE_MODE, MODE_CHANGE_DELAY);
    g_isRecording = false;
}
bool initCamera() {
    for (uint8_t i = 0; i < INIT_MAX_RETRIES; i++) {
        if (getDeviceInfo()) {
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
        delay(INIT_RETRY_DELAY_MS);
    }
    return false;
}


// ── State ─────────────────────────────────────────────────────────────────────
static NimBLEServer*         pServer    = nullptr;
static NimBLECharacteristic* pAckChar   = nullptr;
static bool                  connected  = false;
static bool                  shouldRecord = false;

struct {
    unsigned long now;
    unsigned long camera;
    unsigned long battery;
    unsigned long alive;
} timestamps;
CameraState camState = CameraState::OFF;

char macStr[18];
char macLastTwo[6];


void clientMsg(const char* fmt, ...) {
    char msgbuf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msgbuf, sizeof(msgbuf), fmt, args);
    va_end(args);
    Serial.printf("[CLIENT %s - %s] %s\n", macLastTwo, toString(camState), msgbuf);
}

void notifyState() {
    if (!connected || !pAckChar) return;
    uint8_t val = static_cast<uint8_t>(camState);
    pAckChar->setValue(&val, 1);
    pAckChar->notify();
    timestamps.alive = timestamps.now;
    clientMsg("State notified");
}
void getMacBeforeInit() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);   // get the BLE MAC specifically
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(macLastTwo, sizeof(macLastTwo), "%02X%02X", mac[4], mac[5]);
    clientMsg("MAC: %s", macStr);
}
void changeTo(CameraState nextState){
    if (camState == nextState) return;
    // Serial.printf("[CLIENT] Going from %s to %s\n", toString(camState), toString(nextState));
    clientMsg("Going to %s",  toString(nextState));
    timestamps.camera=timestamps.now;
    camState=nextState;
    notifyState();
}
// void send_msg(uint8_t msg){
//     if (!connected) return;
//     clientMsg("Sent 0x%02X: %s", msg, toString(msg));
//     pAckChar->setValue(&msg, 1);
//     pAckChar->notify();
//     timestamps.alive = timestamps.now;
// }
// void send_ACK(CMD cmd){send_msg(ACK(cmd));}
// void send_CONF(CMD cmd){send_msg(CONF(cmd));}
void updateState(){
    switch (camState){
        case CameraState::OFF:{
            if (shouldRecord) {
                // send_CONF(CMD::START);
                changeTo(CameraState::WAIT_FOR_BOOT);
                break;
            }
            break;
        }
        case CameraState::WAIT_FOR_BOOT:{
            if (!shouldRecord) changeTo(CameraState::STOPPING); break;
            if (initCamera())changeTo(CameraState::BOOTING);break;
            recoverCamera();
            break;
        }
        case CameraState::BOOTING:{
            if (!shouldRecord) changeTo(CameraState::STOPPING); break;
            if ((timestamps.now - timestamps.camera)>1000);{
                startRecording();
                changeTo(CameraState::RECORDING);
            }
            break;
        }
        case CameraState::RECORDING:{
            if (!shouldRecord) {
                // send_CONF(CMD::STOP);
                changeTo(CameraState::STOPPING);
                break;
            }

            break;
        }
        case CameraState::STOPPING:{
            if (shouldRecord) {
                // send_CONF(CMD::START);
                changeTo(CameraState::BOOTING);
                break;
            }
            stopRecording();
            if (!initCamera()) changeTo(CameraState::STOPPED);

            break;
        }
        case CameraState::STOPPED:{
            if (shouldRecord) {
                // send_CONF(CMD::START);
                changeTo(CameraState::BOOTING);
                break;
            }
            if ((timestamps.now - timestamps.camera)>1000){ 

                changeTo(CameraState::OFF);
            }
            break;

        }
        
        default:{
            changeTo(CameraState::OFF);
            break;
        }
    }
}

// ── Your task logic ───────────────────────────────────────────────────────────
void startTask() {
    shouldRecord = true;
    
    clientMsg("Recording requested.");
    // if (camState == CameraState::BOOTING){
    //         // send_CONF(CMD::START);
    //         return;
    //     }
    // if (camState == CameraState::RECORDING){
    //     // send_CONF(CMD::START);
    //     // delay(1000);
    //     // send_ACK(CMD::START);
    //     return;
    // }

    // changeTo(CameraState::BOOTING);


    // TODO: your real work here (e.g. start a motor, begin sampling)
}

void stopTask() {
    shouldRecord = false;
    clientMsg("Recording stop requested.");
    // if (camState == CameraState::STOPPING){
    //         send_CONF(CMD::STOP);
    //         return;
    //     }
    // if (camState == CameraState::STOPPED){
    //     // send_CONF(CMD::STOP);
    //     // delay(1000);
    //     send_ACK(CMD::STOP);
    //     return;
    // }
    // changeTo(CameraState::STOPPING);
    // TODO: your real cleanup here
}

// ── GATT command characteristic callback ──────────────────────────────────────
class CmdCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        if (!pChar->getValue().length()) return;
        uint8_t cmd = pChar->getValue()[0];
        clientMsg("Received 0x%02X: %s", cmd, toString(cmd));

        if (cmd == SEND(CMD::START)) {
            startTask();

        } else if (cmd == SEND(CMD::STOP)) {
            stopTask();
        } else if (cmd == SEND(CMD::REPORT)){
            notifyState();
        }
    }
};
void readBatteryPercent(){
    int t_discharge = 5*60*1000;;
    float V = (t_discharge - (timestamps.now % t_discharge)) * (4.2-2.7)/t_discharge + 2.7;
    sysBatt.update(V);
    sysBatt.soc = sysBatt.soc;
};

class TimeSyncCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        if (pChar->getValue().length() < 4) return;

        // Read 4-byte Unix timestamp
        uint32_t unixTime;
        memcpy(&unixTime, pChar->getValue().data(), sizeof(unixTime));

        // Set system time
        struct timeval tv = { .tv_sec = (time_t)unixTime, .tv_usec = 0 };
        settimeofday(&tv, nullptr);

        // Confirm
        struct tm timeinfo;
        localtime_r((time_t*)&unixTime, &timeinfo);
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        clientMsg("Time synced: %s", buf);
    }
};

// ── Server connection callbacks ───────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
        connected = true;
        clientMsg("Host connected.");
        notifyState();
    }
    void onDisconnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo, int reason) override {
        connected = false;
        clientMsg("Host disconnected. Re-advertising...");
        NimBLEDevice::startAdvertising();
    }
};

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD);
    delay(INITIAL_DELAY);
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);

    timestamps.now = millis();
    timestamps.battery = timestamps.now;
    timestamps.camera = timestamps.now;
    clientMsg("Starting...");
    getMacBeforeInit();
    changeTo(CameraState::OFF);

    NimBLEDevice::init(macLastTwo);
    NimBLEDevice::setPower(Max_TX_Power_db);   // max TX power

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

        // Battery Service (standard 0x180F)
    NimBLEService* pBattService = pServer->createService(BATTERY_SERVICE_UUID);

    pBattChar = pBattService->createCharacteristic(
        BATTERY_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    readBatteryPercent();        
    uint8_t pct = sysBatt.soc;
    pBattChar->setValue(&pct, 1);
    // Create service + characteristics
    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    NimBLECharacteristic* pCmdChar = pService->createCharacteristic(
        CMD_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pCmdChar->setCallbacks(new CmdCallback());

    pAckChar = pService->createCharacteristic(
        ACK_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    NimBLECharacteristic* pTimeChar = pService->createCharacteristic(
        TIME_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pTimeChar->setCallbacks(new TimeSyncCallback());

    // Advertise
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->enableScanResponse(true);   

    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(macLastTwo);
    pAdv->setScanResponseData(scanResponse);  // name in scan response
    pAdv->start();

    clientMsg("MAC type: %s", NimBLEDevice::getAddress().isPublic() ? "PUBLIC" : "RANDOM");

    clientMsg("MAC: %s", NimBLEDevice::getAddress().toString().c_str());
    clientMsg("Advertising, waiting for host...");
    delay(1000);
}

void sendBattUpdate(){
    if (timestamps.now - timestamps.battery > BATT_UPDATE_INTERVAL_MS){
        readBatteryPercent();
        timestamps.battery = timestamps.now;
        // Serial.printf("[CLIENT] Battery level: %d%%", sysBatt.soc);
    }
    if (connected && pBattChar && (sysBatt.soc!=battPctPrev)) {
        uint8_t pct = sysBatt.soc;
        battPctPrev = sysBatt.soc;
        pBattChar->setValue(&pct, 1);
        pBattChar->notify();
        timestamps.alive = timestamps.now;
        // Serial.printf("[CLIENT] Battery level sent to host: %d%%", pct);
        clientMsg("Battery level sent to host: %d%%", pct);
        notifyState();
    }
}

void loop() {
    timestamps.now = millis();
    sendBattUpdate();
    updateState();
    
    if ((timestamps.now-timestamps.alive) > (CLIENT_WATCHDOG_MS/4)){notifyState;}
    delay(10);
}