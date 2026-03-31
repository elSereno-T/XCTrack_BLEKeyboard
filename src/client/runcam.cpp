/**
 * RunCam Thumb Pro W — Time Setting Format Tester
 * =================================================
 * Systematically tries every known date/time string format for
 * WRITE_SETTING (cmd 0x13, setting ID 6).
 *
 * After each attempt it immediately probes with GET_DEVICE_INFO to check
 * whether the camera is still alive. If the camera locks up it waits up
 * to 8 seconds for recovery before giving up on that format and moving on.
 *
 * Press 'r' to run all format tests in sequence.
 * Press '0'-'7' to test a single specific format.
 * Press 'i' to check if camera is alive.
 * Press 's'/'x' to start/stop recording (sanity check).
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>
#include <cstdio>

#define RUNCAM_SERIAL   Serial1
#define RUNCAM_BAUD     115200
#define RUNCAM_RX_PIN   20
#define RUNCAM_TX_PIN   21

static const uint8_t RC_HEADER           = 0xCC;
static const uint8_t RC_ERROR_HEADER     = 0x55;
static const uint8_t CMD_GET_DEVICE_INFO = 0x00;
static const uint8_t CMD_CAMERA_CONTROL  = 0x01;
static const uint8_t CMD_WRITE_SETTING   = 0x13;
static const uint8_t ACTION_POWER_BTN    = 0x01;
static const uint8_t ACTION_CHANGE_MODE  = 0x02;
static const uint8_t SETTING_CAMERA_TIME = 6;

static const uint32_t BTN_PRESS_DELAY_MS = 300;
static const uint32_t MODE_CHANGE_DELAY  = 600;
static const uint32_t RX_TIMEOUT_MS      = 600;   // longer timeout for WRITE_SETTING
static const uint32_t RX_INTER_BYTE_MS   = 30;

static bool     g_cameraReady    = false;
static bool     g_isRecording    = false;
static uint16_t g_cameraFeatures = 0;

// ── Candidate time string formats to test ────────────────────────────────────
// Each entry: { label, string_value }
// The null terminator is added automatically by the send function.
struct TimeFormat {
    const char *label;
    const char *value;
};

static const TimeFormat TIME_FORMATS[] = {
    { "YYYY-MM-DD HH:MM:SS",    "2025-03-19 14:30:00" },  // 0 - standard ISO-like
    { "YYYY/MM/DD HH:MM:SS",    "2025/03/19 14:30:00" },  // 1 - slash separators
    { "DD-MM-YYYY HH:MM:SS",    "19-03-2025 14:30:00" },  // 2 - European order
    { "YYYYMMDDHHMMSS",         "20250319143000"       },  // 3 - compact no separators
    { "YYYY-MM-DD",             "2025-03-19"           },  // 4 - date only
    { "HH:MM:SS",               "14:30:00"             },  // 5 - time only
    { "Unix timestamp (str)",   "1742392200"           },  // 6 - Unix epoch as string
    { "YY-MM-DD HH:MM:SS",      "25-03-19 14:30:00"   },  // 7 - short year
};
static const uint8_t NUM_FORMATS = sizeof(TIME_FORMATS) / sizeof(TIME_FORMATS[0]);

// ── Forward declarations ──────────────────────────────────────────────────────
uint8_t  crc8(const uint8_t *data, size_t len);
void     drainRx();
void     sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen);
size_t   receivePacket(uint8_t *buf, size_t maxLen);

enum class RxResult { OK, ERROR_RESPONSE, NO_RESPONSE, BAD_CRC, BAD_HEADER };
RxResult parseResponse(const uint8_t *buf, size_t len, size_t expectedLen);

bool     getDeviceInfo();
bool     waitForRecovery(uint8_t maxAttempts, uint32_t intervalMs);
void     sendCameraControl(uint8_t action, uint32_t postDelayMs);
bool     startRecording();
bool     stopRecording();

enum class WriteResult { SUCCESS, ERROR_RESPONSE, NO_RESPONSE, LOCKUP, BAD_PACKET };
WriteResult tryWriteTime(const char *timeStr);

void     runAllFormats();
void     testFormat(uint8_t idx);
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

RxResult parseResponse(const uint8_t *buf, size_t len, size_t expectedLen) {
    if (len == 0) return RxResult::NO_RESPONSE;
    if (buf[0] == RC_ERROR_HEADER) {
        if (len >= 5 && buf[len-1] == crc8(buf, len-1))
            Serial.printf("[WARN] Camera error 0x55: code=0x%02X\n", buf[3]);
        else
            Serial.println("[WARN] Camera error 0x55 (malformed)");
        return RxResult::ERROR_RESPONSE;
    }
    if (buf[0] != RC_HEADER)  return RxResult::BAD_HEADER;
    if (len < expectedLen)    return RxResult::NO_RESPONSE;
    if (buf[len-1] != crc8(buf, len-1)) return RxResult::BAD_CRC;
    return RxResult::OK;
}

// ── GET_DEVICE_INFO ───────────────────────────────────────────────────────────
bool getDeviceInfo() {
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);
    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (parseResponse(resp, n, 5) != RxResult::OK) return false;
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    Serial.printf("[INFO] Camera alive. Protocol v%u  Features=0x%04X\n",
                  resp[1], g_cameraFeatures);
    return true;
}

// ── Wait for camera to recover after a lockup ─────────────────────────────────
bool waitForRecovery(uint8_t maxAttempts = 8, uint32_t intervalMs = 1000) {
    Serial.println("[WARN] Waiting for camera recovery...");
    for (uint8_t i = 0; i < maxAttempts; i++) {
        delay(intervalMs);
        Serial.printf("[WARN]   probe %u/%u\n", i + 1, maxAttempts);
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

// ── Try a single WRITE_SETTING with the given time string ────────────────────
WriteResult tryWriteTime(const char *timeStr) {
    size_t strLen     = strlen(timeStr) + 1;   // include null terminator
    size_t payloadLen = 1 + strLen;
    uint8_t payload[32];
    if (payloadLen > sizeof(payload)) return WriteResult::BAD_PACKET;
    payload[0] = SETTING_CAMERA_TIME;
    memcpy(payload + 1, timeStr, strLen);

    sendPacket(CMD_WRITE_SETTING, payload, payloadLen);

    // Response: [0xCC | result(0=OK) | update_menu | crc8] = 4 bytes
    // OR:       [0x55 | 0x05 | 0xFF | error_code | crc8]  = 5 bytes (error)
    uint8_t resp[8];
    size_t n = receivePacket(resp, sizeof(resp));

    if (n == 0) {
        // No response at all — could be lockup, could be intentional (fire&forget)
        // Probe immediately to distinguish
        Serial.println("[TEST] No response received — probing camera...");
        delay(200);
        if (getDeviceInfo()) {
            Serial.println("[TEST] Camera still alive after no-response → might be fire&forget");
            return WriteResult::NO_RESPONSE;
        } else {
            Serial.println("[TEST] Camera not responding → LOCKUP");
            return WriteResult::LOCKUP;
        }
    }

    RxResult r = parseResponse(resp, n, 4);

    if (r == RxResult::ERROR_RESPONSE) return WriteResult::ERROR_RESPONSE;

    if (r == RxResult::OK) {
        uint8_t resultCode = resp[1];
        if (resultCode == 0) {
            Serial.println("[TEST] *** SUCCESS: result code = 0x00 ***");
            return WriteResult::SUCCESS;
        } else {
            Serial.printf("[TEST] Write rejected by camera: result=0x%02X\n", resultCode);
            return WriteResult::ERROR_RESPONSE;
        }
    }

    // Unexpected response — check if alive
    Serial.println("[TEST] Unexpected response — probing camera...");
    delay(200);
    if (!getDeviceInfo()) {
        Serial.println("[TEST] Camera not responding → LOCKUP");
        return WriteResult::LOCKUP;
    }
    return WriteResult::ERROR_RESPONSE;
}

// ── Test a single format by index ────────────────────────────────────────────
void testFormat(uint8_t idx) {
    if (idx >= NUM_FORMATS) {
        Serial.printf("[ERR] Invalid format index %u (max %u)\n", idx, NUM_FORMATS - 1);
        return;
    }
    const TimeFormat &f = TIME_FORMATS[idx];
    Serial.printf("\n[TEST] Format %u: %s\n", idx, f.label);
    Serial.printf("[TEST] Value: \"%s\" (%u bytes incl. null)\n",
                  f.value, (unsigned)strlen(f.value) + 1);

    WriteResult result = tryWriteTime(f.value);

    switch (result) {
        case WriteResult::SUCCESS:
            Serial.printf("[TEST] RESULT: SUCCESS ✓ — format \"%s\" works!\n", f.label);
            break;
        case WriteResult::ERROR_RESPONSE:
            Serial.printf("[TEST] RESULT: REJECTED — camera said no (0x55 or non-zero result)\n");
            break;
        case WriteResult::NO_RESPONSE:
            Serial.printf("[TEST] RESULT: NO RESPONSE — camera accepted silently (check if time changed)\n");
            break;
        case WriteResult::LOCKUP:
            Serial.printf("[TEST] RESULT: LOCKUP — camera stopped responding\n");
            waitForRecovery();
            break;
        case WriteResult::BAD_PACKET:
            Serial.println("[TEST] RESULT: BAD_PACKET — string too long for buffer");
            break;
    }
}

// ── Run all formats in sequence ───────────────────────────────────────────────
void runAllFormats() {
    Serial.println("\n[TEST] ========================================");
    Serial.println("[TEST] Running all time format tests...");
    Serial.println("[TEST] ========================================");

    for (uint8_t i = 0; i < NUM_FORMATS; i++) {
        // Check camera is alive before each test
        Serial.printf("\n[TEST] Pre-test %u: checking camera alive...\n", i);
        if (!getDeviceInfo()) {
            Serial.println("[TEST] Camera not responding — waiting for recovery before continuing");
            if (!waitForRecovery()) {
                Serial.println("[TEST] Cannot recover — aborting test run. Power cycle camera.");
                return;
            }
        }

        testFormat(i);
        delay(500);  // breathing room between tests
    }

    Serial.println("\n[TEST] ========================================");
    Serial.println("[TEST] All formats tested.");
    Serial.println("[TEST] ========================================");
}

// ── Init ──────────────────────────────────────────────────────────────────────
bool initCamera() {
    Serial.println("[INFO] Probing camera...");
    for (uint8_t i = 1; i <= 10; i++) {
        Serial.printf("[INFO] Attempt %u/10\n", i);
        if (getDeviceInfo()) {
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
        delay(1000);
    }
    Serial.println("[ERR] Camera not found");
    return false;
}

// ── Entry points ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W — Time Format Tester ===");
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);
    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);
    initCamera();

    Serial.println();
    Serial.println("[INFO] Commands:");
    Serial.println("  r   = run ALL format tests in sequence");
    for (uint8_t i = 0; i < NUM_FORMATS; i++) {
        Serial.printf("  %u   = test format: %s\n", i, TIME_FORMATS[i].label);
    }
    Serial.println("  i   = check camera alive");
    Serial.println("  s/x = start/stop recording (sanity check)");
    Serial.println("  w   = wait for recovery (if camera locked up)");
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 'r': runAllFormats();  break;
            case 'i':
                Serial.println("[CMD] GET_DEVICE_INFO");
                getDeviceInfo();
                break;
            case 's': startRecording(); break;
            case 'x': stopRecording();  break;
            case 'w': waitForRecovery(); break;
            default:
                if (c >= '0' && c < '0' + NUM_FORMATS) {
                    testFormat(c - '0');
                }
                break;
        }
    }
}