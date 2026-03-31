/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller (v7 - final)
 * ===========================================================
 * Verified behaviour for firmware v1, features=0x0077:
 *
 *  WORKS:
 *   - GET_DEVICE_INFO  (0x00) — responds with 0xCC header, features 0x0077
 *   - CAMERA_CONTROL   (0x01) — ACTION_POWER_BTN (0x01) toggles start/stop
 *   - CAMERA_CONTROL   (0x01) — ACTION_CHANGE_MODE (0x02) cycles video↔QR
 *
 *  DOES NOT WORK (camera returns 0x55 error or locks up):
 *   - READ_SETTING_DETAIL (0x11) for setting ID 6 → 0x55 0x05 0xFF 0x02 (not supported)
 *   - WRITE_SETTING       (0x13) for setting ID 6 → locks up UART parser
 *   The camera advertises SETTINGS_ACCESS (bit4) and DISPLAYPORT (bit5) but
 *   does not implement them for time/date. Do not send these commands.
 *
 *  ERROR RESPONSE FORMAT (0x55 header):
 *   [0x55 | 0x05 | 0xFF | error_code | crc8]
 *   This is a valid CRC-confirmed rejection. The camera is alive when it sends this.
 *
 * LED states:
 *   Solid red       = standby (ready to record)
 *   Slow red flash  = recording
 *   Green           = QR/settings mode
 *   Off             = powered down
 *
 * Wiring (XIAO ESP32-C3 ↔ RunCam 1.25mm 4P):
 *   RunCam TX → GPIO20 (UART1 RX)
 *   RunCam RX → GPIO21 (UART1 TX)
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
static const uint8_t RC_ERROR_HEADER     = 0x55;  // camera-side error response header
static const uint8_t CMD_GET_DEVICE_INFO = 0x00;
static const uint8_t CMD_CAMERA_CONTROL  = 0x01;

// Action IDs for CMD_CAMERA_CONTROL
static const uint8_t ACTION_POWER_BTN    = 0x01;  // toggle start/stop recording
static const uint8_t ACTION_CHANGE_MODE  = 0x02;  // cycle video ↔ QR/settings mode

// Feature flags (GET_DEVICE_INFO response bytes 2+3, little-endian)
static const uint16_t FEAT_POWER_BTN     = (1 << 0);
static const uint16_t FEAT_WIFI_BTN      = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE   = (1 << 2);
static const uint16_t FEAT_5KEY_OSD      = (1 << 3);
static const uint16_t FEAT_SETTINGS      = (1 << 4);
static const uint16_t FEAT_DISPLAYPORT   = (1 << 5);
static const uint16_t FEAT_START_REC     = (1 << 6);
static const uint16_t FEAT_STOP_REC      = (1 << 7);

// ── Timing ────────────────────────────────────────────────────────────────────
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

enum class RxResult { OK, ERROR_RESPONSE, NO_RESPONSE, BAD_CRC, BAD_HEADER };
RxResult parseResponse(const uint8_t *buf, size_t len, size_t expectedLen);

bool    getDeviceInfo();
bool    recoverCamera();
void    sendCameraControl(uint8_t action, uint32_t postDelayMs);
bool    startRecording();
bool    stopRecording();
void    toggleRecording();
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

// Parse a received packet, handling both normal (0xCC) and error (0x55) responses.
RxResult parseResponse(const uint8_t *buf, size_t len, size_t expectedLen) {
    if (len == 0) return RxResult::NO_RESPONSE;

    // Error response from camera: [0x55 | 0x05 | 0xFF | error_code | crc]
    if (buf[0] == RC_ERROR_HEADER) {
        if (len >= 5 && buf[len-1] == crc8(buf, len-1)) {
            Serial.printf("[WARN] Camera error response: code=0x%02X (command not supported)\n",
                          buf[3]);
        } else {
            Serial.println("[WARN] Camera error response (malformed)");
        }
        return RxResult::ERROR_RESPONSE;
    }

    if (buf[0] != RC_HEADER)  return RxResult::BAD_HEADER;
    if (len < expectedLen)    return RxResult::NO_RESPONSE;
    if (buf[len-1] != crc8(buf, len-1)) return RxResult::BAD_CRC;
    return RxResult::OK;
}

// ── GET_DEVICE_INFO (0x00) ────────────────────────────────────────────────────
bool getDeviceInfo() {
    Serial.println("[CMD] GET_DEVICE_INFO");
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);

    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (parseResponse(resp, n, 5) != RxResult::OK) {
        Serial.println("[WARN] GET_DEVICE_INFO failed");
        return false;
    }

    g_protocolVer    = resp[1];
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    Serial.printf("[INFO] Protocol v%u  Features=0x%04X\n", g_protocolVer, g_cameraFeatures);
    if (g_cameraFeatures & FEAT_POWER_BTN)   Serial.println("  + POWER_BTN");
    if (g_cameraFeatures & FEAT_WIFI_BTN)    Serial.println("  + WIFI_BTN");
    if (g_cameraFeatures & FEAT_CHANGE_MODE) Serial.println("  + CHANGE_MODE");
    if (g_cameraFeatures & FEAT_5KEY_OSD)    Serial.println("  + 5KEY_OSD");
    if (g_cameraFeatures & FEAT_SETTINGS)    Serial.println("  + SETTINGS (advertised but not implemented for time/date)");
    if (g_cameraFeatures & FEAT_DISPLAYPORT) Serial.println("  + DISPLAYPORT (advertised but not implemented)");
    if (g_cameraFeatures & FEAT_START_REC)   Serial.println("  + START_REC (advertised; use POWER_BTN instead)");
    if (g_cameraFeatures & FEAT_STOP_REC)    Serial.println("  + STOP_REC");
    return true;
}

// ── Recovery after lockup ─────────────────────────────────────────────────────
bool recoverCamera() {
    Serial.println("[WARN] Attempting recovery...");
    for (uint8_t i = 0; i < 8; i++) {
        delay(1000);
        Serial.printf("[WARN] Recovery probe %u/8\n", i + 1);
        if (getDeviceInfo()) {
            Serial.println("[INFO] Camera recovered");
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
    }
    Serial.println("[ERR] Recovery failed — power cycle required");
    g_cameraReady = false;
    return false;
}

// ── Camera control — fire and forget ─────────────────────────────────────────
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

// ── Recording ─────────────────────────────────────────────────────────────────
bool startRecording() {
    if (g_isRecording) { Serial.println("[WARN] Already recording"); return false; }
    Serial.println("[CMD] Start recording");
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = true;
    Serial.println("[INFO] Recording STARTED (LED: slow red flash)");
    return true;
}

bool stopRecording() {
    if (!g_isRecording) { Serial.println("[WARN] Not recording"); return false; }
    Serial.println("[CMD] Stop recording");
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = false;
    Serial.println("[INFO] Recording STOPPED (LED: solid red)");
    return true;
}

void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
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

// ── Public API ────────────────────────────────────────────────────────────────
// These are the functions you should call from your application code:
//
//   initCamera()       — call once after power-on, after a 3s boot delay
//   startRecording()   — start video
//   stopRecording()    — stop video
//   toggleRecording()  — start if idle, stop if recording
//   getDeviceInfo()    — re-query features (also useful as a keepalive probe)
//   recoverCamera()    — call if camera stops responding
//
// NOTE: Date/time setting (WRITE_SETTING, cmd 0x13) is NOT supported by the
// Thumb Pro W firmware. The camera returns a 0x55 error response for
// READ_SETTING_DETAIL on setting ID 6, and sending WRITE_SETTING locks up
// the UART parser until the camera is power-cycled. Use the RunCam app
// via WiFi to set the date/time instead.

// ── Entry points ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W v7 ===");
    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);
    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);
    initCamera();
    Serial.println("[INFO] Commands: s=start  x=stop  t=toggle  i=info  m=mode  r=recover");
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 's': startRecording();  break;
            case 'x': stopRecording();   break;
            case 't': toggleRecording(); break;
            case 'i': getDeviceInfo();   break;
            case 'm':
                Serial.println("[CMD] CHANGE_MODE");
                sendCameraControl(ACTION_CHANGE_MODE, MODE_CHANGE_DELAY);
                g_isRecording = false;
                break;
            case 'r': recoverCamera();   break;
            default: break;
        }
    }
}