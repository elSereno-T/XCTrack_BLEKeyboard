#include <Arduino.h>
#include <NimBLEDevice.h>
#include <shared.h>
#include <time.h>

// At top of file, alongside your other state vars:
Battery sysBatt;

uint8_t battPctPrev;
static NimBLECharacteristic* pBattChar  = nullptr;   // set during setup()
#define BATT_UPDATE_INTERVAL_MS         10000         // every 5 seconds

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




void clientMsg(const char* fmt, ...) {
    char msgbuf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msgbuf, sizeof(msgbuf), fmt, args);
    va_end(args);
    Serial.printf("[CLIENT] [%s] %s\n", toString(camState), msgbuf);
}

void changeTo(CameraState nextState){
    if (camState == nextState) return;
    // Serial.printf("[CLIENT] Going from %s to %s\n", toString(camState), toString(nextState));
    clientMsg("Going to %s",  toString(nextState));
    timestamps.camera=timestamps.now;
    camState=nextState;
}
void send_msg(uint8_t msg){
    if (!connected) return;
    clientMsg("Sent 0x%02X: %s", msg, toString(msg));
    pAckChar->setValue(&msg, 1);
    pAckChar->notify();
    timestamps.alive = timestamps.now;
}
void send_ACK(CMD cmd){send_msg(ACK(cmd));}
void send_CONF(CMD cmd){send_msg(CONF(cmd));}
void updateState(){
    switch (camState){
        case CameraState::OFF:{
            if (shouldRecord) {
                send_CONF(CMD::START);
                changeTo(CameraState::BOOTING);
                break;
            }
            break;
        }
        case CameraState::BOOTING:{
            if (!shouldRecord) {
                send_CONF(CMD::STOP);
                changeTo(CameraState::STOPPING);
                break;
            }
            if ((timestamps.now - timestamps.camera)>2000){
                send_ACK(CMD::START);
                changeTo(CameraState::RECORDING);
            }
            break;
        }
        case CameraState::RECORDING:{
            if (!shouldRecord) {
                send_CONF(CMD::STOP);
                changeTo(CameraState::STOPPING);
                break;
            }

            break;
        }
        case CameraState::STOPPING:{
            if (shouldRecord) {
                send_CONF(CMD::START);
                changeTo(CameraState::BOOTING);
                break;
            }
            if ((timestamps.now - timestamps.camera)>3000){ 
                send_ACK(CMD::STOP);
                changeTo(CameraState::STOPPED);
            }

            break;
        }
        case CameraState::STOPPED:{
            if (shouldRecord) {
                send_CONF(CMD::START);
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
    if (camState == CameraState::BOOTING){
            send_CONF(CMD::START);
            return;
        }
    if (camState == CameraState::RECORDING){
        // send_CONF(CMD::START);
        // delay(1000);
        send_ACK(CMD::START);
        return;
    }

    // changeTo(CameraState::BOOTING);


    // TODO: your real work here (e.g. start a motor, begin sampling)
}

void stopTask() {
    shouldRecord = false;
    clientMsg("Recording stop requested.");
    if (camState == CameraState::STOPPING){
            send_CONF(CMD::STOP);
            return;
        }
    if (camState == CameraState::STOPPED){
        // send_CONF(CMD::STOP);
        // delay(1000);
        send_ACK(CMD::STOP);
        return;
    }
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
    timestamps.now = millis();
    timestamps.battery = timestamps.now;
    timestamps.camera = timestamps.now;
    clientMsg("Starting...");
    changeTo(CameraState::OFF);

    NimBLEDevice::init("");
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
    pAdv->enableScanResponse(false);
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
        // Serial.printf("[CLIENT] Battery level sent to host: %d%%", pct);
    }
}

void loop() {
    timestamps.now = millis();
    sendBattUpdate();
    updateState();
    
    if ((timestamps.now-timestamps.alive) > (CLIENT_WATCHDOG_MS/4)){send_msg(AWAKE);}
    delay(10);
}