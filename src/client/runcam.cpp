/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller (v2)
 * ====================================================
 * Key fixes over v1:
 *  - CHANGE_MODE (0x02) is NOT blindly sent before start/stop.
 *    On the Thumb Pro W it cycles through modes; sending it when already
 *    in video mode causes it to switch to QR/settings mode, where the
 *    POWER_BTN is interpreted as "power off" — that was the original bug.
 *  - Proper inter-command delays matching ArduPilot timing constants.
 *  - drainRx() clears stale bytes before every send.
 *  - SIMULATE_POWER_BTN (0x01) is the correct toggle for the Thumb Pro W
 *    (ArduPilot Hybrid type). Explicit START/STOP_REC (0x03/0x04) are used
 *    only if the camera advertises them in its feature flags.
 *
 * Wiring (XIAO ESP32-C3 ↔ RunCam Thumb Pro W 1.25mm 4P connector):
 *   RunCam TX  →  GPIO20 (UART1 RX)
 *   RunCam RX  →  GPIO21 (UART1 TX)
 *   RunCam GND →  GND
 *   RunCam 5V  →  5V  (do NOT use the ESP32's 3.3V pin)
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
static const uint8_t RC_HEADER            = 0xCC;

// Command IDs
static const uint8_t CMD_GET_DEVICE_INFO  = 0x00;
static const uint8_t CMD_CAMERA_CONTROL   = 0x01;
static const uint8_t CMD_WRITE_SETTING    = 0x13;

// Camera control action IDs
static const uint8_t ACTION_WIFI_BTN      = 0x00;
static const uint8_t ACTION_POWER_BTN     = 0x01;  // Thumb Pro W: toggles record/stop
static const uint8_t ACTION_CHANGE_MODE   = 0x02;  // cycles between video/photo/QR mode
static const uint8_t ACTION_START_REC     = 0x03;  // explicit start (not all models)
static const uint8_t ACTION_STOP_REC      = 0x04;  // explicit stop  (not all models)

// Feature flags from GET_DEVICE_INFO (uint16_t, little-endian)
static const uint16_t FEAT_POWER_BTN      = (1 << 0);
static const uint16_t FEAT_WIFI_BTN       = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE    = (1 << 2);
static const uint16_t FEAT_5KEY_OSD       = (1 << 3);
static const uint16_t FEAT_SETTINGS       = (1 << 4);
static const uint16_t FEAT_DISPLAYPORT    = (1 << 5);
static const uint16_t FEAT_START_REC      = (1 << 6);
static const uint16_t FEAT_STOP_REC       = (1 << 7);

// Setting IDs
static const uint8_t SETTING_CAMERA_TIME  = 6;  // STRING "YYYY-MM-DD HH:MM:SS"

// ── Timing ────────────────────────────────────────────────────────────────────
// ArduPilot uses 300ms for Hybrid/Thumb Pro button activation delay.
// Too short and commands get out of sync.
static const uint32_t BTN_PRESS_DELAY_MS   = 300;
static const uint32_t MODE_CHANGE_DELAY_MS = 600;   // longer wait after CHANGE_MODE
static const uint32_t RX_TIMEOUT_MS        = 500;
static const uint32_t RX_INTER_BYTE_MS     = 50;
static const uint32_t INIT_RETRY_DELAY_MS  = 1000;
static const uint8_t  INIT_MAX_RETRIES     = 10;

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
void    cycleMode();
bool    setCameraTime(const char *timeStr);
bool    setCameraTimeFromTm(const struct tm *t);
bool    initCamera();

// ── CRC-8 (poly 0xD5, init 0x00) ─────────────────────────────────────────────
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

// ── Drain stale RX bytes ──────────────────────────────────────────────────────
// Call before sending any new packet to avoid confusing responses.
void drainRx() {
    uint32_t deadline = millis() + 20;
    while (millis() < deadline) {
        if (RUNCAM_SERIAL.available()) {
            RUNCAM_SERIAL.read();
            deadline = millis() + 5;
        }
    }
}

// ── Build and send [0xCC | cmd | payload... | crc8] ──────────────────────────
void sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen) {
    uint8_t buf[64];
    size_t totalLen = 2 + payloadLen;
    if (totalLen + 1 > sizeof(buf)) {
        Serial.println("[ERR] Packet too large");
        return;
    }
    buf[0] = RC_HEADER;
    buf[1] = cmd;
    for (size_t i = 0; i < payloadLen; i++) buf[2 + i] = payload[i];
    buf[totalLen] = crc8(buf, totalLen);

    drainRx();
    RUNCAM_SERIAL.write(buf, totalLen + 1);
    RUNCAM_SERIAL.flush();

    Serial.printf("[TX] cmd=0x%02X  payload=%u B  crc=0x%02X\n",
                  cmd, (unsigned)payloadLen, buf[totalLen]);
}

// ── Read response bytes ───────────────────────────────────────────────────────
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
        Serial.printf("[RX] %u bytes:", (unsigned)count);
        for (size_t i = 0; i < count; i++) Serial.printf(" 0x%02X", buf[i]);
        Serial.println();
    } else {
        Serial.println("[RX] (no response)");
    }
    return count;
}

bool validateResponse(const uint8_t *buf, size_t len) {
    if (len < 2) { Serial.println("[ERR] Too short"); return false; }
    if (buf[0] != RC_HEADER) {
        Serial.printf("[ERR] Bad header 0x%02X\n", buf[0]); return false;
    }
    uint8_t expected = crc8(buf, len - 1);
    if (buf[len - 1] != expected) {
        Serial.printf("[ERR] CRC: got 0x%02X expected 0x%02X\n", buf[len-1], expected);
        return false;
    }
    return true;
}

// ── GET_DEVICE_INFO (0x00) ────────────────────────────────────────────────────
// Response: [0xCC | proto_ver | feat_lo | feat_hi | crc8] = 5 bytes
bool getDeviceInfo() {
    Serial.println("[INFO] GET_DEVICE_INFO");
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);

    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (n < 5 || !validateResponse(resp, n)) {
        Serial.println("[WARN] No valid response");
        return false;
    }

    g_protocolVer    = resp[1];
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);

    Serial.printf("[INFO] Protocol v%u  Features=0x%04X\n",
                  g_protocolVer, g_cameraFeatures);
    if (g_cameraFeatures & FEAT_POWER_BTN)   Serial.println("  + POWER_BTN");
    if (g_cameraFeatures & FEAT_WIFI_BTN)    Serial.println("  + WIFI_BTN");
    if (g_cameraFeatures & FEAT_CHANGE_MODE) Serial.println("  + CHANGE_MODE");
    if (g_cameraFeatures & FEAT_5KEY_OSD)    Serial.println("  + 5KEY_OSD");
    if (g_cameraFeatures & FEAT_SETTINGS)    Serial.println("  + SETTINGS_ACCESS");
    if (g_cameraFeatures & FEAT_DISPLAYPORT) Serial.println("  + DISPLAYPORT");
    if (g_cameraFeatures & FEAT_START_REC)   Serial.println("  + START_REC");
    if (g_cameraFeatures & FEAT_STOP_REC)    Serial.println("  + STOP_REC");
    return true;
}

// ── CAMERA_CONTROL (0x01) — fire and forget ───────────────────────────────────
// No ACK expected. postDelayMs gives the camera time to execute the action.
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

// ── Start recording ───────────────────────────────────────────────────────────
// The Thumb Pro W boots into video-standby mode.
// POWER_BTN in standby  → starts recording (LED goes solid red)
// POWER_BTN while recording → stops recording (LED returns to standby blink)
// DO NOT send CHANGE_MODE before this — doing so when already in video mode
// cycles to QR/settings mode, where POWER_BTN means "power off".
bool startRecording() {
    if (g_isRecording) {
        Serial.println("[WARN] Already recording");
        return false;
    }
    Serial.println("[INFO] Start recording...");
    if (g_cameraFeatures & FEAT_START_REC) {
        sendCameraControl(ACTION_START_REC);
    } else {
        sendCameraControl(ACTION_POWER_BTN);
    }
    g_isRecording = true;
    Serial.println("[INFO] Recording STARTED");
    return true;
}

// ── Stop recording ────────────────────────────────────────────────────────────
bool stopRecording() {
    if (!g_isRecording) {
        Serial.println("[WARN] Not recording");
        return false;
    }
    Serial.println("[INFO] Stop recording...");
    if (g_cameraFeatures & FEAT_STOP_REC) {
        sendCameraControl(ACTION_STOP_REC);
    } else {
        sendCameraControl(ACTION_POWER_BTN);
    }
    g_isRecording = false;
    Serial.println("[INFO] Recording STOPPED");
    return true;
}

void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
}

// ── Cycle mode (use with caution) ─────────────────────────────────────────────
// Each call advances through: video standby → QR/settings → video standby → ...
// Only useful if you know you're in the wrong mode. Do not call before
// start/stop unless you're certain the camera has drifted out of video mode.
void cycleMode() {
    Serial.println("[INFO] CHANGE_MODE (cycles to next mode)");
    sendCameraControl(ACTION_CHANGE_MODE, MODE_CHANGE_DELAY_MS);
    // Cycling mode invalidates our recording state assumption — reset it.
    g_isRecording = false;
}

// ── WRITE_SETTING (0x13) — camera clock ──────────────────────────────────────
// Format: "YYYY-MM-DD HH:MM:SS"
// Response: [0xCC | result(0=OK) | update_menu | crc8] = 4 bytes
bool setCameraTime(const char *timeStr) {
    if (!(g_cameraFeatures & FEAT_SETTINGS)) {
        Serial.println("[WARN] SETTINGS_ACCESS not advertised — trying anyway");
    }
    size_t strLen     = strlen(timeStr) + 1;
    size_t payloadLen = 1 + strLen;
    uint8_t payload[32];
    if (payloadLen > sizeof(payload)) return false;
    payload[0] = SETTING_CAMERA_TIME;
    memcpy(payload + 1, timeStr, strLen);

    Serial.printf("[INFO] Set time: %s\n", timeStr);
    sendPacket(CMD_WRITE_SETTING, payload, payloadLen);

    uint8_t resp[4];
    size_t n = receivePacket(resp, sizeof(resp));
    if (n < 4 || !validateResponse(resp, n)) {
        Serial.println("[WARN] No valid response to WRITE_SETTING");
        return false;
    }
    if (resp[1] == 0) { Serial.println("[INFO] Time set OK"); return true; }
    Serial.printf("[ERR] WRITE_SETTING failed, code=0x%02X\n", resp[1]);
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
            g_isRecording = false;  // camera boots in standby
            return true;
        }
        delay(INIT_RETRY_DELAY_MS);
    }
    Serial.println("[ERR] Camera not found");
    return false;
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W v2 ===");

    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);

    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);

    initCamera();

    // Uncomment to sync time after init:
    // setCameraTime("2025-03-19 14:30:00");

    Serial.println("[INFO] Commands:");
    Serial.println("  s = start recording");
    Serial.println("  x = stop recording");
    Serial.println("  t = toggle recording");
    Serial.println("  i = get device info");
    Serial.println("  d = set camera time");
    Serial.println("  m = cycle mode (caution: use only if camera is stuck)");
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
            case 'm': cycleMode();                           break;
            default:  break;
        }
    }
}