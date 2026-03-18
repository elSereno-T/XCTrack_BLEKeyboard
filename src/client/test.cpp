#include <Arduino.h>
#include <NimBLEDevice.h>
#include <shared.h>

static NimBLECharacteristic* pAckChar = nullptr;
static bool connected = false;

class CmdCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        uint8_t cmd = pChar->getValue()[0];
        Serial.printf("[CLIENT] Received CMD: 0x%02X\n", cmd);

        uint8_t ack = (cmd == CMD_START) ? ACK_STARTED : ACK_STOPPED;
        pAckChar->setValue(&ack, 1);
        pAckChar->notify();
        Serial.printf("[CLIENT] Sent ACK: 0x%02X\n", ack);
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
        connected = true;
        Serial.println("[CLIENT] Host connected.");
    }
    void onDisconnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo, int reason) override {
        connected = false;
        Serial.println("[CLIENT] Host disconnected, re-advertising...");
        NimBLEDevice::startAdvertising();
    }
};

void setup() {
    Serial.begin(BAUD);
    delay(INITIAL_DELAY);
    Serial.println("[CLIENT] Starting...");

    NimBLEDevice::init(CLIENT_NAME);
    NimBLEDevice::setPower(9);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

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

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->enableScanResponse(true);          // enable scan response
    // pAdv->setScanResponseData(NimBLEAdvertisementData().setName(CLIENT_NAME) ); // name in scan response
    
    NimBLEAdvertisementData scanResponse;
    scanResponse.setName(CLIENT_NAME);
    pAdv->setScanResponseData(scanResponse);  // name in scan response

    pAdv->start();

    Serial.println("[CLIENT] Advertising...");
}

void loop() {
    delay(10);
}