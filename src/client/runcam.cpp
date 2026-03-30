/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller (v4 - final)
 * ===========================================================
 * Verified behaviour for features=0x0077:
 *
 *   START_REC (bit6) SET,  STOP_REC (bit7) NOT SET
 *   → ACTION_START_REC (0x03) acts as a TOGGLE: start when idle, stop when recording
 *   → ACTION_POWER_BTN (0x01) is the TRUE power button — DO NOT use for recording
 *   → ACTION_CHANGE_MODE (0x02) cycles between video and QR/settings mode
 *
 * LED states (Thumb Pro W):
 *   Solid red           = standby (ready to record)
 *   Slow red flash      = recording
 *   Green solid/flash   = QR/settings mode
 *   Off                 = powered down
 *
 * Wiring (XIAO ESP32-C3 ↔ RunCam 1.25mm 4P):
 *   RunCam TX  →  GPIO20 (UART1 RX)
 *   RunCam RX  →  GPIO21 (UART1 TX)
 *   RunCam GND →  GND
 *   RunCam 5V  →  5V supply (NOT 3.3V)
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

// Action IDs for CMD_CAMERA_CONTROL
static const uint8_t ACTION_WIFI_BTN     = 0x00;
static const uint8_t ACTION_POWER_BTN    = 0x01;  // TRUE power on/off — do NOT use for recording
static const uint8_t ACTION_CHANGE_MODE  = 0x02;  // cycles video ↔ QR/settings
static const uint8_t ACTION_START_REC    = 0x03;  // toggles start/stop on Thumb Pro W
static const uint8_t ACTION_STOP_REC     = 0x04;  // only use if FEAT_STOP_REC is set

// Feature flags (GET_DEVICE_INFO response byte 2+3, little-endian)
static const uint16_t FEAT_POWER_BTN     = (1 << 0);
static const uint16_t FEAT_WIFI_BTN      = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE   = (1 << 2);
static const uint16_t FEAT_5KEY_OSD      = (1 << 3);
static const uint16_t FEAT_SETTINGS      = (1 << 4);
static const uint16_t FEAT_DISPLAYPORT   = (1 << 5);
static const uint16_t FEAT_START_REC     = (1 << 6);
static const uint16_t FEAT_STOP_REC      = (1 << 7);

// Setting IDs
static const uint8_t SETTING_CAMERA_TIME = 6;  // STRING "YYYY-MM-DD HH:MM:SS"

// ── Timing ────────────────────────────────────────────────────────────────────
static const uint32_t BTN_PRESS_DELAY_MS  = 300;   // post-command settle time
static const uint32_t MODE_CHANGE_DELAY   = 600;   // longer wait after CHANGE_MODE
static const uint32_t RX_TIMEOUT_MS       = 500;
static const uint32_t RX_INTER_BYTE_MS    = 50;
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

// ── CRC-8 poly 0xD5 (matches Betaflight's crc8HighFirst) ─────────────────────
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

// ── Drain stale RX bytes before sending ──────────────────────────────────────
void drainRx() {
    uint32_t deadline = millis() + 20;
    while (millis() < deadline) {
        if (RUNCAM_SERIAL.available()) {
            RUNCAM_SERIAL.read();
            deadline = millis() + 5;
        }
    }
}

// ── Build and transmit packet ─────────────────────────────────────────────────
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

// ── Read response ─────────────────────────────────────────────────────────────
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
        Serial.println("[RX] (no response)");
    }
    return count;
}

bool validateResponse(const uint8_t *buf, size_t len) {
    if (len < 2) return false;
    if (buf[0] != RC_HEADER) return false;
    return buf[len - 1] == crc8(buf, len - 1);
}

// ── GET_DEVICE_INFO (0x00) ────────────────────────────────────────────────────
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

// ── Camera control — fire and forget ─────────────────────────────────────────
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

// ── Recording control ─────────────────────────────────────────────────────────
/**
 * Recording command selection (based on features=0x0077):
 *
 *   START_REC set, STOP_REC not set:
 *     → Use ACTION_START_REC (0x03) as a toggle for BOTH start and stop.
 *       The camera treats it as: "start if idle, stop if recording."
 *
 *   START_REC AND STOP_REC both set:
 *     → Use ACTION_START_REC (0x03) to start, ACTION_STOP_REC (0x04) to stop.
 *
 *   Neither set:
 *     → Fallback to ACTION_WIFI_BTN (0x00) as toggle.
 *       (POWER_BTN is the real power button — never use it for recording.)
 */
bool startRecording() {
    if (g_isRecording) { Serial.println("[WARN] Already recording"); return false; }
    Serial.println("[CMD] Start recording");

    if (g_cameraFeatures & FEAT_START_REC) {
        // 0x03 toggles: starts when idle
        sendCameraControl(ACTION_START_REC);
    } else {
        // Fallback: wifi button acts as shutter on some models
        Serial.println("[WARN] START_REC not advertised, trying WIFI_BTN");
        sendCameraControl(ACTION_WIFI_BTN);
    }

    g_isRecording = true;
    Serial.println("[INFO] Recording STARTED");
    return true;
}

bool stopRecording() {
    if (!g_isRecording) { Serial.println("[WARN] Not recording"); return false; }
    Serial.println("[CMD] Stop recording");

    if (g_cameraFeatures & FEAT_STOP_REC) {
        // Explicit stop supported
        sendCameraControl(ACTION_STOP_REC);
    } else if (g_cameraFeatures & FEAT_START_REC) {
        // START_REC toggles: stops when recording
        sendCameraControl(ACTION_START_REC);
    } else {
        sendCameraControl(ACTION_WIFI_BTN);
    }

    g_isRecording = false;
    Serial.println("[INFO] Recording STOPPED");
    return true;
}

void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
}

// ── WRITE_SETTING (0x13) — set camera clock ──────────────────────────────────
/**
 * Sets the camera RTC. Format: "YYYY-MM-DD HH:MM:SS"
 * Only call after confirming FEAT_SETTINGS (bit4) is set.
 * Response: [0xCC | result(0=OK) | update_menu | crc8] = 4 bytes.
 */
bool setCameraTime(const char *timeStr) {
    if (!(g_cameraFeatures & FEAT_SETTINGS)) {
        Serial.println("[WARN] SETTINGS not advertised — trying anyway");
    }
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

// ── Entry points ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W v4 ===");
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);
    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);
    initCamera();

    // Optionally sync time on boot — replace with NTP/RTC values:
    // setCameraTime("2025-03-19 14:30:00");

    Serial.println("[INFO] Commands: s=start  x=stop  t=toggle  i=info  d=set-time  m=change-mode");
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