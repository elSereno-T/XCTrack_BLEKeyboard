/**
 * RunCam Thumb Pro W — Read Time Setting
 * =======================================
 * Tries two different commands to read the current camera time:
 *
 *  1. GET_SETTINGS (0x10) with parent ID = 0 (root scan)
 *     → returns all top-level settings as [id | name\0 | value\0] triplets
 *     → the value for ID 6 will reveal the exact format the camera uses
 *
 *  2. GET_SETTINGS (0x10) with parent ID = 6 directly
 *     → in case ID 6 is a folder with sub-settings
 *
 *  3. READ_SETTING_DETAIL (0x11) with ID 6
 *     → already known to return 0x55 error, but included for completeness
 *
 * Also includes the working recording control from v7.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>
#include <cstdio>

#define RUNCAM_SERIAL   Serial1
#define RUNCAM_BAUD     115200
#define RUNCAM_RX_PIN   20
#define RUNCAM_TX_PIN   21

static const uint8_t RC_HEADER              = 0xCC;
static const uint8_t RC_ERROR_HEADER        = 0x55;
static const uint8_t CMD_GET_DEVICE_INFO    = 0x00;
static const uint8_t CMD_CAMERA_CONTROL     = 0x01;
static const uint8_t CMD_GET_SETTINGS       = 0x10;
static const uint8_t CMD_READ_SETTING_DETAIL = 0x11;
static const uint8_t CMD_WRITE_SETTING      = 0x13;

static const uint8_t ACTION_POWER_BTN       = 0x01;
static const uint8_t ACTION_CHANGE_MODE     = 0x02;
static const uint8_t SETTING_CAMERA_TIME    = 6;

static const uint32_t BTN_PRESS_DELAY_MS    = 300;
static const uint32_t MODE_CHANGE_DELAY     = 600;
// Longer timeout for settings responses — they can be chunked
static const uint32_t RX_TIMEOUT_MS         = 800;
static const uint32_t RX_INTER_BYTE_MS      = 40;

static bool     g_isRecording    = false;
static uint16_t g_cameraFeatures = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
uint8_t  crc8(const uint8_t *data, size_t len);
void     drainRx();
void     sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen);
size_t   receivePacket(uint8_t *buf, size_t maxLen);
void     printPacketHex(const char *prefix, const uint8_t *buf, size_t len);
bool     isErrorResponse(const uint8_t *buf, size_t len);
bool     isValidResponse(const uint8_t *buf, size_t len);
bool     getDeviceInfo();
void     sendCameraControl(uint8_t action, uint32_t postDelayMs);
bool     startRecording();
bool     stopRecording();
void     getSettingsAtID(uint8_t parentId);
void     readSettingDetail(uint8_t settingId);
void     scanAllSettings();
bool     initCamera();

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
    uint8_t buf[64];
    size_t totalLen = 2 + payloadLen;
    if (totalLen + 1 > sizeof(buf)) { Serial.println("[ERR] Packet too large"); return; }
    buf[0] = RC_HEADER;
    buf[1] = cmd;
    for (size_t i = 0; i < payloadLen; i++) buf[2 + i] = payload[i];
    buf[totalLen] = crc8(buf, totalLen);
    printPacketHex("[TX]", buf, totalLen + 1);
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
    if (count > 0) printPacketHex("[RX]", buf, count);
    else           Serial.println("[RX] (none)");
    return count;
}

void printPacketHex(const char *prefix, const uint8_t *buf, size_t len) {
    Serial.print(prefix);
    for (size_t i = 0; i < len; i++) Serial.printf(" 0x%02X", buf[i]);
    // Also print any printable ASCII inline
    Serial.print("  |");
    for (size_t i = 0; i < len; i++) {
        char c = (char)buf[i];
        Serial.print((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    Serial.println("|");
}

bool isErrorResponse(const uint8_t *buf, size_t len) {
    if (len < 2) return false;
    return buf[0] == RC_ERROR_HEADER;
}

bool isValidResponse(const uint8_t *buf, size_t len) {
    if (len < 2) return false;
    if (buf[0] != RC_HEADER) return false;
    return buf[len - 1] == crc8(buf, len - 1);
}

// ── GET_DEVICE_INFO ───────────────────────────────────────────────────────────
bool getDeviceInfo() {
    Serial.println("[CMD] GET_DEVICE_INFO");
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);
    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (!isValidResponse(resp, n)) { Serial.println("[WARN] No valid response"); return false; }
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    Serial.printf("[INFO] Protocol v%u  Features=0x%04X\n", resp[1], g_cameraFeatures);
    return true;
}

void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

bool startRecording() {
    if (g_isRecording) { Serial.println("[WARN] Already recording"); return false; }
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = true;
    Serial.println("[INFO] Recording STARTED");
    return true;
}

bool stopRecording() {
    if (!g_isRecording) { Serial.println("[WARN] Not recording"); return false; }
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = false;
    Serial.println("[INFO] Recording STOPPED");
    return true;
}

// ── GET_SETTINGS (0x10) ───────────────────────────────────────────────────────
/**
 * Requests sub-settings under parentId, reading all chunks.
 *
 * Request:  [0xCC | 0x10 | parent_id | chunk_index | crc]
 * Response: [0xCC | remaining_chunks | data_length | {id, name\0, value\0}... | crc]
 *
 * Prints each setting's ID, name, and current value string.
 * The value of SETTINGID_DISP_CAMERA_TIME (ID 6) reveals the time format.
 */
void getSettingsAtID(uint8_t parentId) {
    Serial.printf("\n[CMD] GET_SETTINGS parent_id=%u\n", parentId);

    uint8_t chunk = 0;
    uint8_t remaining = 0;

    do {
        uint8_t payload[2] = { parentId, chunk };
        sendPacket(CMD_GET_SETTINGS, payload, 2);

        // Response can be large — use a generous buffer
        uint8_t resp[128];
        size_t n = receivePacket(resp, sizeof(resp));

        if (n == 0) {
            Serial.println("[WARN] No response");
            return;
        }
        if (isErrorResponse(resp, n)) {
            Serial.printf("[WARN] Error response (0x55) for parent_id=%u chunk=%u\n",
                          parentId, chunk);
            return;
        }
        if (!isValidResponse(resp, n)) {
            Serial.println("[WARN] Invalid response (bad header or CRC)");
            return;
        }

        // Parse response structure:
        // resp[0] = 0xCC (header)
        // resp[1] = remaining_chunks
        // resp[2] = data_length (from resp[3] to resp[n-2], i.e. excluding header, remaining, dataLen, crc)
        // resp[3..n-2] = settings data
        // resp[n-1] = crc

        remaining   = resp[1];
        uint8_t dataLen = resp[2];

        Serial.printf("[INFO] Chunk %u, remaining=%u, data_length=%u\n",
                      chunk, remaining, dataLen);

        // Walk through settings data: each entry is [setting_id | name\0 | value\0]
        size_t pos = 3;  // start of data (after header, remaining, dataLen)
        size_t dataEnd = 3 + dataLen;
        if (dataEnd > n - 1) dataEnd = n - 1;  // guard against overflow, stop before CRC

        while (pos < dataEnd) {
            // Setting ID
            uint8_t settingId = resp[pos++];
            if (pos >= dataEnd) break;

            // Setting name (null-terminated string)
            char name[64] = {};
            size_t ni = 0;
            while (pos < dataEnd && resp[pos] != 0x00 && ni < sizeof(name) - 1) {
                name[ni++] = (char)resp[pos++];
            }
            if (pos < dataEnd) pos++;  // consume null terminator

            // Setting value (null-terminated string)
            char value[64] = {};
            size_t vi = 0;
            while (pos < dataEnd && resp[pos] != 0x00 && vi < sizeof(value) - 1) {
                value[vi++] = (char)resp[pos++];
            }
            if (pos < dataEnd) pos++;  // consume null terminator

            Serial.printf("  [ID=%2u] %-30s = \"%s\"\n", settingId, name, value);

            // Highlight the time setting
            if (settingId == SETTING_CAMERA_TIME) {
                Serial.printf("  ^^^ THIS IS THE CAMERA TIME (ID 6) — format is: \"%s\"\n", value);
            }
        }

        chunk++;
    } while (remaining > 0);
}

// ── READ_SETTING_DETAIL (0x11) ────────────────────────────────────────────────
/**
 * Requests the detail of a single setting (type, min/max, current value, etc).
 * For STRING type: returns max_string_size.
 * Already known to return 0x55 for ID 6, but try anyway in case other IDs work.
 */
void readSettingDetail(uint8_t settingId) {
    Serial.printf("\n[CMD] READ_SETTING_DETAIL id=%u\n", settingId);

    uint8_t payload[2] = { settingId, 0x00 };
    sendPacket(CMD_READ_SETTING_DETAIL, payload, 2);

    uint8_t resp[64];
    size_t n = receivePacket(resp, sizeof(resp));

    if (n == 0) { Serial.println("[WARN] No response"); return; }

    if (isErrorResponse(resp, n)) {
        if (n >= 5) Serial.printf("[WARN] Error 0x55: error_code=0x%02X\n", resp[3]);
        else        Serial.println("[WARN] Error 0x55 (short)");
        return;
    }

    if (!isValidResponse(resp, n)) { Serial.println("[WARN] Invalid response"); return; }

    // resp[1] = remaining_chunks
    // resp[2] = data_length
    // resp[3] = setting_type
    uint8_t settingType = resp[3];
    const char *typeName = "unknown";
    switch (settingType) {
        case 0:  typeName = "UINT8";          break;
        case 1:  typeName = "INT8";           break;
        case 2:  typeName = "UINT16";         break;
        case 3:  typeName = "INT16";          break;
        case 8:  typeName = "FLOAT";          break;
        case 9:  typeName = "TEXT_SELECTION"; break;
        case 10: typeName = "STRING";         break;
        case 11: typeName = "FOLDER";         break;
        case 12: typeName = "INFO";           break;
    }
    Serial.printf("[INFO] setting_type = 0x%02X (%s)\n", settingType, typeName);

    if (settingType == 10) {  // STRING
        // Find null terminator of current value starting at resp[4]
        size_t pos = 4;
        char curVal[64] = {};
        size_t vi = 0;
        while (pos < n - 1 && resp[pos] != 0x00 && vi < sizeof(curVal) - 1) {
            curVal[vi++] = (char)resp[pos++];
        }
        if (pos < n - 1) pos++;  // consume null
        Serial.printf("[INFO] current value = \"%s\"\n", curVal);

        if (pos < n - 1) {
            uint8_t maxStrSize = resp[pos];
            Serial.printf("[INFO] max_string_size = %u\n", maxStrSize);
        }
    }
}

// ── Scan root settings then ID 6 specifically ────────────────────────────────
void scanAllSettings() {
    Serial.println("\n========================================");
    Serial.println("Scanning root settings (parent_id=0)...");
    Serial.println("========================================");
    getSettingsAtID(0);

    Serial.println("\n========================================");
    Serial.println("Scanning ID 6 as parent (sub-settings)...");
    Serial.println("========================================");
    getSettingsAtID(6);

    Serial.println("\n========================================");
    Serial.println("READ_SETTING_DETAIL for ID 6...");
    Serial.println("========================================");
    readSettingDetail(6);

    // Also try IDs 0-10 with READ_SETTING_DETAIL to see what responds
    Serial.println("\n========================================");
    Serial.println("READ_SETTING_DETAIL for IDs 0-10...");
    Serial.println("========================================");
    for (uint8_t id = 0; id <= 10; id++) {
        Serial.printf("\n--- ID %u ---\n", id);
        readSettingDetail(id);
        delay(200);
        // Quick alive check every 3 IDs
        if (id % 3 == 2) {
            if (!getDeviceInfo()) {
                Serial.println("[WARN] Camera not responding — waiting 3s...");
                delay(3000);
                getDeviceInfo();
            }
        }
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────
bool initCamera() {
    for (uint8_t i = 1; i <= 10; i++) {
        Serial.printf("[INFO] Probe %u/10\n", i);
        if (getDeviceInfo()) return true;
        delay(1000);
    }
    return false;
}

// ── Entry points ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W — Setting Reader ===");
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);
    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);
    initCamera();

    Serial.println("\n[INFO] Commands:");
    Serial.println("  a = scan ALL settings (root + ID6 + detail IDs 0-10)");
    Serial.println("  g = GET_SETTINGS parent_id=0  (root scan)");
    Serial.println("  6 = GET_SETTINGS parent_id=6  (time sub-settings)");
    Serial.println("  d = READ_SETTING_DETAIL id=6  (time detail)");
    Serial.println("  i = GET_DEVICE_INFO");
    Serial.println("  s = start recording");
    Serial.println("  x = stop recording");
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'a': scanAllSettings();          break;
            case 'g': getSettingsAtID(0);         break;
            case '6': getSettingsAtID(6);         break;
            case 'd': readSettingDetail(6);       break;
            case 'i': getDeviceInfo();            break;
            case 's': startRecording();           break;
            case 'x': stopRecording();            break;
            default: break;
        }
    }
}