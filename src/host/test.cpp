#include <Arduino.h>
#include <NimBLEDevice.h>
#include <shared.h>

uint16_t NextTryDelta = 5000;
#define TIMEOUT_ACK_MS 5000

static NimBLEClient*               pClient  = nullptr;
static NimBLERemoteCharacteristic* pCmd     = nullptr;
static bool                        ackReceived = false;

const char* knownMacs[] = { "98:3d:ae:ab:e2:7a" , "98:3d:ae:ac:92:9a" };
const int   knownMacCount = sizeof(knownMacs) / sizeof(knownMacs[0]);


struct BleClient {
    NimBLEClient*               pClient     = nullptr;
    NimBLERemoteCharacteristic* pCmd        = nullptr;
    NimBLERemoteCharacteristic* pAck        = nullptr;
    NimBLERemoteCharacteristic* pBatt       = nullptr;
    bool                        ackReceived = false;
    bool                        waitingForAck = false;
    uint8_t                     battPercent = 255;   // 255 = unknown
    bool                        newbattvalue = false;
    bool                        connected   = false;
    unsigned long               nextTry     = 0;
    unsigned long               commandSent = 0;         
};

static BleClient clients[knownMacCount];

struct {
    unsigned long now;
    unsigned long camera;
    unsigned long system;
} timestamps;

uint8_t NEXT = CMD_START;

void sendCmd(uint8_t cmd) {
    Serial.printf("[HOST] Sent CMD: 0x%02X\n", cmd);
    for (int i = 0; i < knownMacCount; i++) {
        if (!clients[i].connected) continue;
        clients[i].ackReceived = false;
        clients[i].waitingForAck = true;
        clients[i].commandSent = timestamps.now;
        clients[i].pCmd->writeValue(&cmd, 1, false);
    }
}

bool waitForAck() {
    bool any = false;
    for (int i = 0; i < knownMacCount; i++) {
        if (!clients[i].waitingForAck) continue;
        if (clients[i].ackReceived){
            Serial.printf("[HOST] ACK received from client %d.\n", i);
            clients[i].waitingForAck = false;
            any = true;
        } else if (((timestamps.now - clients[i].commandSent) > TIMEOUT_ACK_MS)){
            Serial.printf("[HOST] ACK timeout for client %d.\n", i);
            clients[i].waitingForAck = false;

        }
    }
    return any;
}
// ── Client callbacks ──────────────────────────────────────────────────────────
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        // find which index this client is
        for (int i = 0; i < knownMacCount; i++) {
            if (clients[i].pClient == pClient) {
                Serial.printf("[HOST] Client %d connected.\n", i);
                return;
            }
        }
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        for (int i = 0; i < knownMacCount; i++) {
            if (clients[i].pClient == pClient) {
                Serial.printf("[HOST] Client %d disconnected! Reason: %d\n", i, reason);
                // Clear characteristics — they're invalid after disconnect
                clients[i].pCmd        = nullptr;
                clients[i].pAck        = nullptr;
                clients[i].pBatt       = nullptr;
                clients[i].ackReceived = false;
                clients[i].battPercent = 255;
                clients[i].connected   = false;
                clients[i].nextTry     = timestamps.now;
                // Don't delete pClient here — do it when reconnecting
                return;
            }
        }
    }
};

static ClientCallbacks clientCallbacks; 
bool connectToClient(int idx) {
     
    Serial.printf("[HOST] directly connecting to %s...\n", knownMacs[idx]);

    NimBLEClient* pC = NimBLEDevice::createClient();
        pC->setClientCallbacks(&clientCallbacks, false);  // false = don't delete on disconnect
        pC->setConnectTimeout(2000);  // short timeout — don't wait long

    if (!pC->connect(NimBLEAddress(knownMacs[idx], BLE_ADDR_PUBLIC))) {
        Serial.printf("[HOST] Direct connect failed: %s\n", knownMacs[idx]);
        NimBLEDevice::deleteClient(pC);
        return false;
    }
    clients[idx].connected = true;
    Serial.printf("[HOST] Connected to: %s\n", knownMacs[idx]);

    // ── CMD / ACK service ─────────────────────────────────────────────────────
    NimBLERemoteService* pSvc = pC->getService(SERVICE_UUID);
    if (!pSvc) {
        Serial.printf("[HOST] Service not found on %s\n", knownMacs[idx]);
        pC->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic* pCmd = pSvc->getCharacteristic(CMD_CHAR_UUID);
    NimBLERemoteCharacteristic* pAck = pSvc->getCharacteristic(ACK_CHAR_UUID);
    if (!pCmd || !pAck) {
        Serial.printf("[HOST] Characteristics not found on %s\n", knownMacs[idx]);
        pC->disconnect();
        return false;
    }

    clients[idx].pClient = pC;
    clients[idx].pCmd    = pCmd;
    clients[idx].pAck    = pAck;

    // Subscribe to ACK notifications
    pAck->subscribe(true,
        [idx](NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
            if (!len) return;
            Serial.printf("[HOST] Client %d ACK: 0x%02X\n", idx, data[0]);
            clients[idx].ackReceived = true;
        }
    );

    // ── Battery service (optional) ────────────────────────────────────────────
    NimBLERemoteService* pBattSvc = pC->getService(BATTERY_SERVICE_UUID);
    if (pBattSvc) {
        NimBLERemoteCharacteristic* pBatt =
            pBattSvc->getCharacteristic(BATTERY_CHAR_UUID);
        if (pBatt) {
            clients[idx].pBatt       = pBatt;
            clients[idx].battPercent = pBatt->readValue<uint8_t>();
            Serial.printf("[HOST] Client %d battery: %d%%\n",
                          idx, clients[idx].battPercent);

            pBatt->subscribe(true,
                [idx](NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
                    if (!len) return;
                    clients[idx].battPercent = data[0];
                    Serial.printf("[HOST] Client %d battery update: %d%%\n",
                                  idx, clients[idx].battPercent);
                }
            );
        }
    }
    return true;

}
void callConnectToClient(int idx){
        if (timestamps.now<clients[idx].nextTry || clients[idx].connected) return;
        clients[idx].connected = connectToClient(idx);
        if (!clients[idx].connected) clients[idx].nextTry =timestamps.now +  NextTryDelta;
}
void ConnectAll(){
    for (int i = 0; i < knownMacCount; i++) {

        if (clients[i].pClient && !clients[i].pClient->isConnected()) {
            Serial.printf("[HOST] Client %d lost, reconnecting...\n", i);
            NimBLEDevice::deleteClient(clients[i].pClient);
            clients[i].pClient = nullptr;
            clients[i].connected = false;
            clients[i].nextTry = timestamps.now-1;
        }
        callConnectToClient(i);
    }
}
void setup() {
    Serial.begin(BAUD);
    delay(INITIAL_DELAY);
    Serial.println("[HOST] Starting...");
    timestamps.now=millis();
    timestamps.camera = timestamps.now;

    NimBLEDevice::init(HOST_NAME);
    NimBLEDevice::setPower(Max_TX_Power_db);   // max TX power
}

void loop() {    
    timestamps.now=millis();
    ConnectAll();

    if ((timestamps.now-timestamps.camera)>10000){

        Serial.printf("[HOST] send camera command\n");
        sendCmd(NEXT);
        timestamps.camera = timestamps.now;
        NEXT = (NEXT==CMD_START) ? CMD_STOP : CMD_START;
    }
    waitForAck();
    delay(10);
}