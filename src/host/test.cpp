#include <Arduino.h>
#include <NimBLEDevice.h>
#include "../shared.h"

#define SCAN_DURATION_MS  8000

static NimBLEClient*               pClient  = nullptr;
static NimBLERemoteCharacteristic* pCmd     = nullptr;
static bool                        ackReceived = false;

void sendCmd(uint8_t cmd) {
    ackReceived = false;
    pCmd->writeValue(&cmd, 1, false);
    Serial.printf("[HOST] Sent CMD: 0x%02X\n", cmd);
}

bool waitForAck(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (!ackReceived) {
        if (millis() - start > timeoutMs) {
            Serial.println("[HOST] ACK timeout!");
            return false;
        }
        delay(10);
    }
    Serial.println("[HOST] ACK received.");
    return true;
}

bool connectToClient() {
    Serial.println("[HOST] Scanning...");

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(45);
    pScan->setWindow(15);
    pScan->setMaxResults(0xFF);

    // Non-blocking start — duration 0 means scan indefinitely until stopped
    pScan->start(0, false);

    // Wait manually for 8 seconds
    Serial.println("[HOST] Waiting 8s...");
    uint32_t scanStart = millis();
    while (millis() - scanStart < 8000) {
        Serial.printf("[HOST] Scanning... %lums elapsed, found %d so far\n",
                      millis() - scanStart,
                      pScan->getResults().getCount());
        delay(1000);
    }

    pScan->stop();
    Serial.println("[HOST] Scan stopped.");

    NimBLEScanResults results = pScan->getResults();
    Serial.printf("[HOST] Total: %d device(s) found.\n", results.getCount());

    for (int i = 0; i < (int)results.getCount(); i++) {
        const NimBLEAdvertisedDevice* dev = results.getDevice(i);
        Serial.printf("  [%d] addr: %s\n", i,
                      dev->getAddress().toString().c_str());
        Serial.printf("       name: '%s'\n", dev->getName().c_str());
        Serial.printf("       RSSI: %d\n", dev->getRSSI());
        Serial.printf("       advertisingService: %d\n",
                      dev->isAdvertisingService(NimBLEUUID(SERVICE_UUID)));
    }

    pScan->clearResults();

    // rest of your connect logic...
    return false; // placeholder for now
}

void setup() {
    Serial.begin(BAUD);
    delay(INITIAL_DELAY);
    Serial.println("[HOST] Starting...");

    NimBLEDevice::init("TestHost");
    NimBLEDevice::setPower(9);

    if (!connectToClient()) {
        Serial.println("[HOST] Could not connect. Halting.");
        while (true) delay(1000);
    }

    // ── Test sequence ─────────────────────────────────────────────────────────
    delay(500);

    Serial.println("[HOST] Sending CMD_START...");
    sendCmd(CMD_START);
    waitForAck(3000);

    delay(2000);

    Serial.println("[HOST] Sending CMD_STOP...");
    sendCmd(CMD_STOP);
    waitForAck(3000);

    Serial.println("[HOST] Test complete.");
}

void loop() {
    delay(100);
}