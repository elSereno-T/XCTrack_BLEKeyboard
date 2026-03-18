#include <Arduino.h>
#include <NimBLEDevice.h>
#include <shared.h>

#define SCAN_DURATION_MS  8000

static NimBLEClient*               pClient  = nullptr;
static NimBLERemoteCharacteristic* pCmd     = nullptr;
static bool                        ackReceived = false;

const char* knownMacs[] = { "98:3d:ae:ab:e2:7a" , "98:3d:ae:ab:e2:9a" };
const int   knownMacCount = sizeof(knownMacs) / sizeof(knownMacs[0]);

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
    bool allConnected = true;
    Serial.println("[HOST] Scanning...");

     // ── Phase 1: try direct connection to known MACs ──────────────────────────
    for (int i = 0; i < knownMacCount; i++) {
        Serial.printf("[HOST] Trying direct connect to %s...\n", knownMacs[i]);

        NimBLEClient* pC = NimBLEDevice::createClient();
        pC->setConnectTimeout(2000);  // short timeout — don't wait long

        if (pC->connect(NimBLEAddress(knownMacs[i], BLE_ADDR_PUBLIC))) {
            Serial.printf("[HOST] Direct connect OK: %s\n", knownMacs[i]);
        } else {
            Serial.printf("[HOST] Direct connect failed: %s, will scan.\n", knownMacs[i]);
            NimBLEDevice::deleteClient(pC);
            allConnected = false;
        }
    }

    if (allConnected) return true;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(45);
    pScan->setWindow(15);
    pScan->setMaxResults(0xff);

    pScan->start(0, false);

    uint32_t scanStart = millis();
    while (pScan->isScanning()) {
        if (millis() - scanStart > 8000) {
            pScan->stop();
            Serial.println("[HOST] Scan timeout.");
        }
        delay(10);
    }

    NimBLEScanResults results = pScan->getResults();
    Serial.printf("[HOST] Total: %d device(s) found.\n", results.getCount());

    // Print all found devices
    for (int i = 0; i < (int)results.getCount(); i++) {
        const NimBLEAdvertisedDevice* dev = results.getDevice(i);
        Serial.printf("  [%d] addr: %s  name: '%s'  RSSI: %d  svc: %d\n",
                      i,
                      dev->getAddress().toString().c_str(),
                      dev->getName().c_str(),
                      dev->getRSSI(),
                      dev->isAdvertisingService(NimBLEUUID(SERVICE_UUID)));
    }

    // ── Match on service UUID ─────────────────────────────────────────────────
    const NimBLEAdvertisedDevice* target = nullptr;
    for (int i = 0; i < (int)results.getCount(); i++) {
        const NimBLEAdvertisedDevice* dev = results.getDevice(i);
        if (dev->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
            target = dev;
            Serial.printf("[HOST] Target found: %s (name: '%s', RSSI: %d)\n",
                          dev->getAddress().toString().c_str(),
                          dev->getName().c_str(),
                          dev->getRSSI());
            break;
        }
    }

    if (!target) {
        Serial.println("[HOST] Client not found.");
        pScan->clearResults();
        return false;
    }

    pClient = NimBLEDevice::createClient();
    if (!pClient->connect(target)) {
        Serial.println("[HOST] Connection failed.");
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        pScan->clearResults();
        return false;
    }

    pScan->clearResults();
    Serial.println("[HOST] Connected.");

    NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
    if (!pSvc) {
        Serial.println("[HOST] Service not found.");
        return false;
    }

    pCmd = pSvc->getCharacteristic(CMD_CHAR_UUID);
    NimBLERemoteCharacteristic* pAck = pSvc->getCharacteristic(ACK_CHAR_UUID);
    if (!pCmd || !pAck) {
        Serial.println("[HOST] Characteristics not found.");
        return false;
    }

    pAck->subscribe(true,
        [](NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
            if (!len) return;
            Serial.printf("[HOST] ACK: 0x%02X\n", data[0]);
            ackReceived = true;
        }
    );

    return true;
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