#include <Arduino.h>
#include <NimBLEDevice.h>
#include <shared.h>

// At top of file, alongside your other state vars:
uint8_t battPct;
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
} timestamps;

void changeState(CameraState nextState){
    if (camState == nextState) return;
    Serial.println("[CLIENT] Going from "+ CameraStateString[camState] + " to " + CameraStateString[nextState]);
    timestamps.camera=timestamps.now;
    camState=nextState;
}
void updateState(){
    switch (camState){
        case CAMERA_OFF:{
            if (shouldRecord) changeState(CAMERA_BOOTING);
            break;
        }
        case CAMERA_BOOTING:{
            if ((timestamps.now - timestamps.camera)>2000){
                
                uint8_t ack = ACK_STARTED;
                pAckChar->setValue(&ack, 1);
                pAckChar->notify();
                Serial.println("[CLIENT] ACK_STARTED sent.");
                changeState(CAMERA_RECORDING);
            }
            break;
        }
        case CAMERA_RECORDING:{
            if (!shouldRecord) changeState(CAMERA_STOPPING);

            break;
        }
        case CAMERA_STOPPING:{
            if ((timestamps.now - timestamps.camera)>1000){ 
                uint8_t ack = ACK_STOPPED;
                pAckChar->setValue(&ack, 1);
                pAckChar->notify();
                Serial.println("[CLIENT] ACK_STOPPED sent.");

                changeState(CAMERA_SHUTDOWN);
            }

            break;
        }
        case CAMERA_SHUTDOWN:{
            if ((timestamps.now - timestamps.camera)>1000){ 

                changeState(CAMERA_OFF);
            }
            break;

        }
        
        default:{
            changeState(CAMERA_OFF);
            break;
        }
    }
}

// ── Your task logic ───────────────────────────────────────────────────────────
void startTask() {
    shouldRecord = true;
    
    Serial.println("[CLIENT] Recording requested.");


    // TODO: your real work here (e.g. start a motor, begin sampling)
}

void stopTask() {
    shouldRecord = false;
    Serial.println("[CLIENT] Recording stop requested.");
    // TODO: your real cleanup here
}

// ── GATT command characteristic callback ──────────────────────────────────────
class CmdCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        if (!pChar->getValue().length()) return;
        uint8_t cmd = pChar->getValue()[0];
        Serial.printf("[CLIENT] Received CMD: 0x%02X\n", cmd);

        if (cmd == CMD_START) {
            startTask();

        } else if (cmd == CMD_STOP) {
            stopTask();
        }
    }
};
void readBatteryPercent(){
    battPct = 100 - (timestamps.now / (5*60*10))%100;
};

// ── Server connection callbacks ───────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
        connected = true;
        Serial.println("[CLIENT] Host connected.");
    }
    void onDisconnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo, int reason) override {
        connected = false;
        Serial.println("[CLIENT] Host disconnected. Re-advertising...");
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
    Serial.println("[CLIENT] Starting...");
    changeState(CAMERA_OFF);

    NimBLEDevice::init(CLIENT_NAME);   // all clients same prefix, host filters by it
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
    uint8_t pct = battPct;
    pBattChar->setValue(&pct, 1);
    pBattService->start();
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

    pService->start();

    // Advertise
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->enableScanResponse(true);
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(CLIENT_NAME);
    pAdv->setScanResponseData(scanResponse);  // name in scan response
    pAdv->start();
    Serial.printf("[CLIENT] MAC type: %s\n", NimBLEDevice::getAddress().isPublic() ? "PUBLIC" : "RANDOM");
    Serial.printf("[CLIENT] MAC: %s\n", NimBLEDevice::getAddress().toString().c_str());
    Serial.println("[CLIENT] Advertising, waiting for host...");
    delay(1000);
}

void sendBattUpdate(){
    if (timestamps.now - timestamps.battery > BATT_UPDATE_INTERVAL_MS){
        readBatteryPercent();
        timestamps.battery = timestamps.now;
        Serial.printf("[CLIENT] Battery level: %d%%\n", battPct);
    }
    if (connected && pBattChar && (battPct!=battPctPrev)) {
        uint8_t pct = battPct;
        battPctPrev = battPct;
        pBattChar->setValue(&pct, 1);
        pBattChar->notify();
        Serial.printf("[CLIENT] Battery level sent to host: %d%%\n", pct);
    }
}

void loop() {
    timestamps.now = millis();
    sendBattUpdate();
    updateState();
    delay(10);
}