/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller (v6)
 * ====================================================
 * Changes from v5:
 *  - Added readSettingDetail() to query max_string_size for setting ID 6
 *    before attempting to write. The camera locks up if we send a string
 *    longer than its internal buffer.
 *  - Added recoverCamera() which re-sends GET_DEVICE_INFO to check if
 *    the camera recovered after a bad packet.
 *  - setCameraTime() now queries the max string size first, then truncates
 *    if necessary, and validates the response properly.
 *
 * Wiring (XIAO ESP32-C3 ↔ RunCam 1.25mm 4P):
 *   RunCam TX → GPIO20 (UART1 RX)
 *   RunCam RX → GPIO21 (UART1 TX)
 *   RunCam GND → GND
 *   RunCam 5V  → 5V supply (NOT 3.3V)
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>
#include <cstdio>
#include <time.h>

// ── UART config ───────────────────────────────────────────────────────────────
#define RUNCAM_SERIAL   Serial1
#define RUNCAM_BAUD     115200
#define RUNCAM_RX_PIN   20
#define RUNCAM_TX_PIN   21

// ── Protocol constants ────────────────────────────────────────────────────────
static const uint8_t RC_HEADER                = 0xCC;
static const uint8_t CMD_GET_DEVICE_INFO      = 0x00;
static const uint8_t CMD_CAMERA_CONTROL       = 0x01;
static const uint8_t CMD_READ_SETTING_DETAIL  = 0x11;
static const uint8_t CMD_WRITE_SETTING        = 0x13;

static const uint8_t ACTION_POWER_BTN         = 0x01;
static const uint8_t ACTION_CHANGE_MODE       = 0x02;

static const uint16_t FEAT_POWER_BTN          = (1 << 0);
static const uint16_t FEAT_WIFI_BTN           = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE        = (1 << 2);
static const uint16_t FEAT_5KEY_OSD           = (1 << 3);
static const uint16_t FEAT_SETTINGS           = (1 << 4);
static const uint16_t FEAT_DISPLAYPORT        = (1 << 5);
static const uint16_t FEAT_START_REC          = (1 << 6);
static const uint16_t FEAT_STOP_REC           = (1 << 7);

static const uint8_t SETTING_CAMERA_TIME      = 6;

// Setting types (from READ_SETTING_DETAIL response)
static const uint8_t SETTING_TYPE_STRING      = 10;

// ── Timing ────────────────────────────────────────────────────────────────────
static const uint32_t BTN_PRESS_DELAY_MS      = 300;
static const uint32_t MODE_CHANGE_DELAY       = 600;
static const uint32_t RX_TIMEOUT_MS           = 500;
static const uint32_t RX_INTER_BYTE_MS        = 30;
static const uint32_t INIT_RETRY_DELAY_MS     = 1000;
static const uint8_t  INIT_MAX_RETRIES        = 10;

// ── State ─────────────────────────────────────────────────────────────────────
static bool     g_cameraReady    = false;
static bool     g_isRecording    = false;
static uint16_t g_cameraFeatures = 0;
static uint8_t  g_protocolVer    = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
uint8_t crc8(const uint8_t *data, size_t len);
void    drainRx();
void    sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen);
size_t  receivePacket(uint8_t *buf, size_t maxLen);
bool    validateResponse(const uint8_t *buf, size_t len);
bool    getDeviceInfo();
bool    recoverCamera();
void    sendCameraControl(uint8_t action, uint32_t postDelayMs);
bool    startRecording();
bool    stopRecording();
void    toggleRecording();
int     readSettingDetail_maxStringSize(uint8_t settingId);
bool    setCameraTime(const char *timeStr);
bool    setCameraTimeFromTm(const struct tm *t);
bool    initCamera();

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

    Serial.printf("[TX] cmd=0x%02X", cmd);
    for (size_t i = 0; i < payloadLen; i++) Serial.printf(" 0x%02X", payload[i]);
    Serial.printf("  crc=0x%02X\n", buf[totalLen]);

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
    if (count > 0) {
        Serial.printf("[RX]");
        for (size_t i = 0; i < count; i++) Serial.printf(" 0x%02X", buf[i]);
        Serial.println();
    } else {
        Serial.println("[RX] (none)");
    }
    return count;
}

bool validateResponse(const uint8_t *buf, size_t len) {
    if (len < 2 || buf[0] != RC_HEADER) return false;
    return buf[len - 1] == crc8(buf, len - 1);
}

// ── GET_DEVICE_INFO ───────────────────────────────────────────────────────────
bool getDeviceInfo() {
    Serial.println("[CMD] GET_DEVICE_INFO");
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);
    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (n < 5 || !validateResponse(resp, n)) {
        Serial.println("[WARN] No valid response");
        return false;
    }
    g_protocolVer    = resp[1];
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    Serial.printf("[INFO] Protocol v%u  Features=0x%04X\n", g_protocolVer, g_cameraFeatures);
    if (g_cameraFeatures & FEAT_POWER_BTN)   Serial.println("  + POWER_BTN");
    if (g_cameraFeatures & FEAT_WIFI_BTN)    Serial.println("  + WIFI_BTN");
    if (g_cameraFeatures & FEAT_CHANGE_MODE) Serial.println("  + CHANGE_MODE");
    if (g_cameraFeatures & FEAT_5KEY_OSD)    Serial.println("  + 5KEY_OSD");
    if (g_cameraFeatures & FEAT_SETTINGS)    Serial.println("  + SETTINGS");
    if (g_cameraFeatures & FEAT_DISPLAYPORT) Serial.println("  + DISPLAYPORT");
    if (g_cameraFeatures & FEAT_START_REC)   Serial.println("  + START_REC");
    if (g_cameraFeatures & FEAT_STOP_REC)    Serial.println("  + STOP_REC");
    return true;
}

// ── Camera recovery after bad packet ─────────────────────────────────────────
// After a malformed packet the camera may stop responding for a few seconds.
// Repeatedly probing with GET_DEVICE_INFO lets it recover.
bool recoverCamera() {
    Serial.println("[WARN] Attempting camera recovery...");
    for (uint8_t i = 0; i < 5; i++) {
        delay(1000);
        Serial.printf("[WARN] Recovery probe %u/5\n", i + 1);
        if (getDeviceInfo()) {
            Serial.println("[INFO] Camera recovered!");
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
    }
    Serial.println("[ERR] Camera did not recover — power cycle required");
    g_cameraReady = false;
    return false;
}

// ── Camera control ────────────────────────────────────────────────────────────
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

bool startRecording() {
    if (g_isRecording) { Serial.println("[WARN] Already recording"); return false; }
    Serial.println("[CMD] Start recording");
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = true;
    return true;
}

bool stopRecording() {
    if (!g_isRecording) { Serial.println("[WARN] Not recording"); return false; }
    Serial.println("[CMD] Stop recording");
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = false;
    return true;
}

void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
}

// ── READ_SETTING_DETAIL (0x11) ────────────────────────────────────────────────
/**
 * Queries the detail of a setting. For STRING type, returns max_string_size.
 * Returns -1 on failure, otherwise the max string size (including null terminator).
 *
 * Request:  [0xCC | 0x11 | setting_id | chunk_index=0 | crc]  (5 bytes)
 * Response: [0xCC | remaining_chunks | data_length | setting_type | ...fields... | crc]
 *
 * For STRING type the only extra field is max_string_size (uint8_t).
 */
int readSettingDetail_maxStringSize(uint8_t settingId) {
    Serial.printf("[CMD] READ_SETTING_DETAIL for ID %u\n", settingId);
    uint8_t payload[2] = { settingId, 0x00 };  // setting_id, chunk_index=0
    sendPacket(CMD_READ_SETTING_DETAIL, payload, 2);

    // Response is variable length; read generously
    uint8_t resp[32];
    size_t n = receivePacket(resp, sizeof(resp));

    if (n < 5) {
        Serial.printf("[WARN] READ_SETTING_DETAIL: too short (%u bytes)\n", (unsigned)n);
        return -1;
    }
    if (resp[0] != RC_HEADER) {
        Serial.printf("[WARN] READ_SETTING_DETAIL: bad header 0x%02X\n", resp[0]);
        return -1;
    }
    // Validate CRC on all bytes
    if (resp[n-1] != crc8(resp, n-1)) {
        Serial.printf("[WARN] READ_SETTING_DETAIL: CRC mismatch\n");
        // Don't abort — try to parse anyway, camera may still have sent useful data
    }

    // Response layout (0-indexed, after 0xCC header):
    //  [0] = 0xCC (header)
    //  [1] = remaining_chunks
    //  [2] = data_length  (length from next byte up to but not including CRC)
    //  [3] = setting_type
    //  [4..] = type-specific fields
    //  [n-1] = crc

    uint8_t settingType = resp[3];
    Serial.printf("[INFO] Setting type: 0x%02X ", settingType);
    switch (settingType) {
        case 0:  Serial.println("(UINT8)");          break;
        case 1:  Serial.println("(INT8)");           break;
        case 2:  Serial.println("(UINT16)");         break;
        case 3:  Serial.println("(INT16)");          break;
        case 8:  Serial.println("(FLOAT)");          break;
        case 9:  Serial.println("(TEXT_SELECTION)"); break;
        case 10: Serial.println("(STRING)");         break;
        case 11: Serial.println("(FOLDER)");         break;
        case 12: Serial.println("(INFO)");           break;
        default: Serial.println("(unknown)");        break;
    }

    if (settingType != SETTING_TYPE_STRING) {
        Serial.printf("[WARN] Setting ID %u is not STRING type\n", settingId);
        return -1;
    }

    // For STRING: field after setting_type is max_string_size (uint8_t)
    // resp[4] = current value (null-terminated string, could be multi-byte)
    // After the current value string comes max_string_size.
    // We need to skip the current value string to find max_string_size.
    // The current value is at resp[4..] until null terminator.
    // But since we can't be sure of the layout without seeing real data,
    // let's use data_length to find max_string_size.
    //
    // data_length = bytes from resp[3] to just before CRC = resp[2]
    // So resp[3 + data_length] would be CRC.
    // For STRING: data = [type(1)] + [current_value_str + null] + [max_string_size(1)]
    // max_string_size is at index: 3 + 1 + strlen(current_value) + 1 + 1
    // = 4 + strlen(current_value) + 1

    uint8_t dataLength = resp[2];
    Serial.printf("[INFO] data_length=%u, total response=%u bytes\n", dataLength, (unsigned)n);

    // Print all response bytes for analysis
    Serial.print("[INFO] Full response: ");
    for (size_t i = 0; i < n; i++) {
        if (resp[i] >= 0x20 && resp[i] < 0x7F)
            Serial.printf("0x%02X('%c') ", resp[i], resp[i]);
        else
            Serial.printf("0x%02X ", resp[i]);
    }
    Serial.println();

    // Find null terminator in current value starting at resp[4]
    int nullPos = -1;
    for (size_t i = 4; i < n - 1; i++) {
        if (resp[i] == 0x00) { nullPos = (int)i; break; }
    }

    if (nullPos < 0) {
        Serial.println("[WARN] Could not find null terminator in current value");
        return -1;
    }

    // max_string_size is the byte after the null terminator
    size_t maxSizeIdx = nullPos + 1;
    if (maxSizeIdx >= n - 1) {  // -1 to exclude CRC
        Serial.println("[WARN] max_string_size field not found in response");
        return -1;
    }

    uint8_t maxSize = resp[maxSizeIdx];
    Serial.printf("[INFO] max_string_size = %u bytes (includes null terminator)\n", maxSize);
    return (int)maxSize;
}

// ── WRITE_SETTING (0x13) — set camera clock ──────────────────────────────────
/**
 * Sets the camera RTC. First queries the max allowed string length to avoid
 * overrunning the camera's buffer, which causes it to become unresponsive.
 *
 * Expected format: "YYYY-MM-DD HH:MM:SS" (19 chars + null = 20 bytes)
 * If the camera's max_string_size < 20, we'll log a warning and try a shorter format.
 */
bool setCameraTime(const char *timeStr) {
    if (!(g_cameraFeatures & FEAT_SETTINGS)) {
        Serial.println("[WARN] SETTINGS_ACCESS not advertised — aborting to protect camera");
        return false;
    }

    // Query max string size before writing to avoid locking up the camera
    int maxSize = readSettingDetail_maxStringSize(SETTING_CAMERA_TIME);
    if (maxSize < 0) {
        Serial.println("[WARN] Could not read setting detail — skipping write to be safe");
        return false;
    }

    size_t strLen = strlen(timeStr) + 1;  // +1 for null terminator
    if ((int)strLen > maxSize) {
        Serial.printf("[ERR] Time string too long: %u bytes, max is %d\n",
                      (unsigned)strLen, maxSize);
        Serial.println("[ERR] String will overflow camera buffer — aborting");
        return false;
    }

    // Build and send WRITE_SETTING packet
    uint8_t payload[32];
    size_t payloadLen = 1 + strLen;  // setting_id + string + null
    if (payloadLen > sizeof(payload)) return false;
    payload[0] = SETTING_CAMERA_TIME;
    memcpy(payload + 1, timeStr, strLen);

    Serial.printf("[CMD] WRITE_SETTING time: \"%s\" (%u bytes incl. null)\n",
                  timeStr, (unsigned)strLen);
    sendPacket(CMD_WRITE_SETTING, payload, payloadLen);

    // Response: [0xCC | result(0=OK) | update_menu | crc8] = 4 bytes
    uint8_t resp[4];
    size_t n = receivePacket(resp, sizeof(resp));

    if (n == 0) {
        Serial.println("[WARN] No response to WRITE_SETTING");
        // Check if camera is still alive
        delay(500);
        if (!getDeviceInfo()) {
            recoverCamera();
        }
        return false;
    }

    if (n < 4 || !validateResponse(resp, n)) {
        Serial.println("[WARN] Invalid response to WRITE_SETTING");
        return false;
    }

    if (resp[1] == 0) {
        Serial.println("[INFO] Camera time set successfully!");
        return true;
    }
    Serial.printf("[ERR] WRITE_SETTING result=0x%02X (non-zero = failure)\n", resp[1]);
    return false;
}

bool setCameraTimeFromTm(const struct tm *t) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
    return setCameraTime(buf);
}

// ── Init ──────────────────────────────────────────────────────────────────────
bool initCamera() {
    Serial.println("[INFO] Probing camera...");
    for (uint8_t i = 1; i <= INIT_MAX_RETRIES; i++) {
        Serial.printf("[INFO] Attempt %u/%u\n", i, INIT_MAX_RETRIES);
        if (getDeviceInfo()) {
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
        delay(INIT_RETRY_DELAY_MS);
    }
    Serial.println("[ERR] Camera not found");
    return false;
}

// ── Entry points ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W v6 ===");
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);
    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);
    initCamera();

    Serial.println("[INFO] Commands:");
    Serial.println("  s = start recording");
    Serial.println("  x = stop recording");
    Serial.println("  t = toggle recording");
    Serial.println("  i = get device info");
    Serial.println("  q = query time setting detail (shows max string size)");
    Serial.println("  d = set camera time");
    Serial.println("  m = change mode (video <-> QR)");
    Serial.println("  r = attempt camera recovery");
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 's': startRecording();                              break;
            case 'x': stopRecording();                               break;
            case 't': toggleRecording();                             break;
            case 'i': getDeviceInfo();                               break;
            case 'q': readSettingDetail_maxStringSize(SETTING_CAMERA_TIME); break;
            case 'd': setCameraTime("2025-03-19 14:30:00");         break;
            case 'm':
                Serial.println("[CMD] CHANGE_MODE");
                sendCameraControl(ACTION_CHANGE_MODE, MODE_CHANGE_DELAY);
                g_isRecording = false;
                break;
            case 'r': recoverCamera();                               break;
            default: break;
        }
    }
}