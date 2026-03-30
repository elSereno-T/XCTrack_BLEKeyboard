/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller (v5)
 * ====================================================
 * Command mapping verified against the Thumb Pro W manual and Betaflight source:
 *
 *   CAMERA POWER (start/stop video) = ACTION_POWER_BTN  (0x01)  ← toggle
 *   CAMERA CHANGE MODE              = ACTION_CHANGE_MODE (0x02)  ← cycles video↔QR
 *
 * The START_REC (0x03) / STOP_REC (0x04) actions in the feature flags (0x0077)
 * appear to be reported by the camera but are NOT the correct commands for the
 * Thumb Pro W — Betaflight uses POWER_BTN for record start/stop on this model.
 *
 * LED states:
 *   Solid red        = standby (powered, ready)
 *   Slow red flash   = recording
 *   Green solid      = QR/settings mode
 *   Off              = powered down
 *
 * Wiring (XIAO ESP32-C3 ↔ RunCam 1.25mm 4P):
 *   RunCam TX  → GPIO20 (UART1 RX)
 *   RunCam RX  → GPIO21 (UART1 TX)
 *   RunCam GND → GND
 *   RunCam 5V  → 5V supply (NOT 3.3V)
 *
 * Baud: 115200, 8N1
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
static const uint8_t RC_HEADER           = 0xCC;
static const uint8_t CMD_GET_DEVICE_INFO = 0x00;
static const uint8_t CMD_CAMERA_CONTROL  = 0x01;
static const uint8_t CMD_WRITE_SETTING   = 0x13;

// Action IDs for CMD_CAMERA_CONTROL (0x01)
static const uint8_t ACTION_WIFI_BTN     = 0x00;  // toggle WiFi
static const uint8_t ACTION_POWER_BTN    = 0x01;  // CAMERA POWER: toggle start/stop recording
static const uint8_t ACTION_CHANGE_MODE  = 0x02;  // CAMERA CHANGE MODE: cycle video ↔ QR
static const uint8_t ACTION_START_REC    = 0x03;  // explicit start (not used for Thumb Pro W)
static const uint8_t ACTION_STOP_REC     = 0x04;  // explicit stop  (not used for Thumb Pro W)

// Feature flags
static const uint16_t FEAT_POWER_BTN     = (1 << 0);
static const uint16_t FEAT_WIFI_BTN      = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE   = (1 << 2);
static const uint16_t FEAT_5KEY_OSD      = (1 << 3);
static const uint16_t FEAT_SETTINGS      = (1 << 4);
static const uint16_t FEAT_DISPLAYPORT   = (1 << 5);
static const uint16_t FEAT_START_REC     = (1 << 6);
static const uint16_t FEAT_STOP_REC      = (1 << 7);

static const uint8_t SETTING_CAMERA_TIME = 6;  // STRING "YYYY-MM-DD HH:MM:SS"

// ── Timing ────────────────────────────────────────────────────────────────────
// 300ms matches Betaflight's rcdevice debounce for the Thumb Pro / Hybrid type.
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

// ── Forward declarations ──────────────────────────────────────────────────────
uint8_t crc8(const uint8_t *data, size_t len);
void    drainRx();
void    sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen);
size_t  receivePacket(uint8_t *buf, size_t maxLen);
bool    validateResponse(const uint8_t *buf, size_t len);
bool    getDeviceInfo();
void    sendCameraControl(uint8_t action, uint32_t postDelayMs);
bool    startRecording();
bool    stopRecording();
void    toggleRecording();
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
    for (size_t i = 0; i < payloadLen; i++) Serial.printf(" arg=0x%02X", payload[i]);
    Serial.printf(" crc=0x%02X\n", buf[totalLen]);

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
        Serial.println("[RX] (none — normal for control commands)");
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

// ── Camera control ────────────────────────────────────────────────────────────
// Control commands send no ACK — the postDelay gives the camera time to act.
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

// ── Recording ─────────────────────────────────────────────────────────────────
// ACTION_POWER_BTN (0x01) = "CAMERA POWER" in Betaflight = start/stop toggle.
// This is what the Thumb Pro W manual maps to the SA switch.
bool startRecording() {
    if (g_isRecording) { Serial.println("[WARN] Already recording"); return false; }
    Serial.println("[CMD] Start recording → ACTION_POWER_BTN");
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = true;
    Serial.println("[INFO] Recording STARTED (LED should flash red slowly)");
    return true;
}

bool stopRecording() {
    if (!g_isRecording) { Serial.println("[WARN] Not recording"); return false; }
    Serial.println("[CMD] Stop recording → ACTION_POWER_BTN");
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = false;
    Serial.println("[INFO] Recording STOPPED (LED should return to solid red)");
    return true;
}

void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
}

// ── WRITE_SETTING — set camera clock ─────────────────────────────────────────
// Format: "YYYY-MM-DD HH:MM:SS"
bool setCameraTime(const char *timeStr) {
    size_t strLen     = strlen(timeStr) + 1;
    size_t payloadLen = 1 + strLen;
    uint8_t payload[32];
    if (payloadLen > sizeof(payload)) return false;
    payload[0] = SETTING_CAMERA_TIME;
    memcpy(payload + 1, timeStr, strLen);

    Serial.printf("[CMD] Set time: %s\n", timeStr);
    sendPacket(CMD_WRITE_SETTING, payload, payloadLen);

    uint8_t resp[4];
    size_t n = receivePacket(resp, sizeof(resp));
    if (n < 4 || !validateResponse(resp, n)) {
        Serial.println("[WARN] No valid response to WRITE_SETTING");
        return false;
    }
    if (resp[1] == 0) { Serial.println("[INFO] Time set OK"); return true; }
    Serial.printf("[ERR] WRITE_SETTING failed, result=0x%02X\n", resp[1]);
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
    Serial.println("\n=== RunCam Thumb Pro W v5 ===");
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);
    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);
    initCamera();

    // Uncomment to sync time on boot (replace with NTP/RTC values):
    // setCameraTime("2025-03-19 14:30:00");

    Serial.println("[INFO] Commands:");
    Serial.println("  s = start recording");
    Serial.println("  x = stop recording");
    Serial.println("  t = toggle recording");
    Serial.println("  i = get device info");
    Serial.println("  d = set camera time");
    Serial.println("  m = change mode (video <-> QR)");
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 's': startRecording();                      break;
            case 'x': stopRecording();                       break;
            case 't': toggleRecording();                     break;
            case 'i': getDeviceInfo();                       break;
            case 'd': setCameraTime("2025-03-19 14:30:00"); break;
            case 'm':
                Serial.println("[CMD] CHANGE_MODE");
                sendCameraControl(ACTION_CHANGE_MODE, MODE_CHANGE_DELAY);
                g_isRecording = false;
                break;
            default: break;
        }
    }
}