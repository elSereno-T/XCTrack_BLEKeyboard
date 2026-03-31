/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller (final)
 * =======================================================
 * Verified command support for firmware v1, features=0x0077:
 *
 *  ✓  GET_DEVICE_INFO  (0x00) — works, returns features=0x0077
 *  ✓  CAMERA_CONTROL   (0x01) ACTION_POWER_BTN   (0x01) — toggles start/stop recording
 *  ✓  CAMERA_CONTROL   (0x01) ACTION_CHANGE_MODE (0x02) — cycles video ↔ QR/settings
 *
 *  ✗  GET_SETTINGS        (0x10) — returns 0x55 error, not implemented
 *  ✗  READ_SETTING_DETAIL (0x11) — returns 0x55 error, not implemented
 *  ✗  WRITE_SETTING       (0x13) — locks up UART parser, do not use
 *
 *  The SETTINGS_ACCESS bit in the feature flags is advertised but the entire
 *  settings subsystem is unimplemented. Every settings command returns the
 *  same blanket error: 0x55 0x05 0xFF 0x02 0x1A (error_code=0x02).
 *  Set date/time via the RunCam WiFi app instead.
 *
 * Error response format (0x55 header):
 *  [0x55 | 0x05 | 0xFF | error_code | crc8]  — camera alive, command rejected
 *
 * LED states:
 *  Solid red       = standby (ready to record)
 *  Slow red flash  = recording
 *  Green           = QR/settings mode
 *  Off             = powered down
 *
 * Wiring (XIAO ESP32-C3 ↔ RunCam 1.25mm 4P):
 *  RunCam TX → GPIO20 (UART1 RX)
 *  RunCam RX → GPIO21 (UART1 TX)
 *  RunCam GND → GND
 *  RunCam 5V  → 5V supply (NOT 3.3V)
 *
 * Baud: 115200, 8N1
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>
#include <cstdio>

// ── UART config ───────────────────────────────────────────────────────────────
#define RUNCAM_SERIAL   Serial1
#define RUNCAM_BAUD     115200
#define RUNCAM_RX_PIN   20
#define RUNCAM_TX_PIN   21

// ── Protocol constants ────────────────────────────────────────────────────────
static const uint8_t RC_HEADER           = 0xCC;
static const uint8_t RC_ERROR_HEADER     = 0x55;
static const uint8_t CMD_GET_DEVICE_INFO = 0x00;
static const uint8_t CMD_CAMERA_CONTROL  = 0x01;

// Action IDs for CMD_CAMERA_CONTROL
static const uint8_t ACTION_POWER_BTN    = 0x01;  // toggles start/stop recording
static const uint8_t ACTION_CHANGE_MODE  = 0x02;  // cycles video ↔ QR/settings

// Feature flags from GET_DEVICE_INFO (little-endian uint16)
static const uint16_t FEAT_POWER_BTN     = (1 << 0);
static const uint16_t FEAT_WIFI_BTN      = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE   = (1 << 2);
static const uint16_t FEAT_5KEY_OSD      = (1 << 3);
static const uint16_t FEAT_SETTINGS      = (1 << 4);  // advertised but NOT implemented
static const uint16_t FEAT_DISPLAYPORT   = (1 << 5);  // advertised but NOT implemented
static const uint16_t FEAT_START_REC     = (1 << 6);  // advertised but use POWER_BTN instead
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
bool    isValidResponse(const uint8_t *buf, size_t len, size_t expectedLen);
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
    uint8_t buf[32];
    size_t totalLen = 2 + payloadLen;
    if (totalLen + 1 > sizeof(buf)) { Serial.println("[ERR] Packet too large"); return; }
    buf[0] = RC_HEADER;
    buf[1] = cmd;
    for (size_t i = 0; i < payloadLen; i++) buf[2 + i] = payload[i];
    buf[totalLen] = crc8(buf, totalLen);
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
    return count;
}

bool isValidResponse(const uint8_t *buf, size_t len, size_t expectedLen) {
    if (len < expectedLen)    return false;
    if (buf[0] != RC_HEADER)  return false;
    return buf[len - 1] == crc8(buf, len - 1);
}

// ── GET_DEVICE_INFO (0x00) ────────────────────────────────────────────────────
bool getDeviceInfo() {
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);
    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));
    if (!isValidResponse(resp, n, 5)) return false;
    g_protocolVer    = resp[1];
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    return true;
}

// ── Recovery ──────────────────────────────────────────────────────────────────
bool recoverCamera() {
    for (uint8_t i = 0; i < 8; i++) {
        delay(1000);
        if (getDeviceInfo()) {
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
    }
    g_cameraReady = false;
    return false;
}

// ── Camera control ────────────────────────────────────────────────────────────
void sendCameraControl(uint8_t action, uint32_t postDelayMs = BTN_PRESS_DELAY_MS) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    delay(postDelayMs);
    drainRx();
}

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Start video recording.
 * Camera must be in standby (solid red LED).
 * Returns false if already recording.
 */
bool startRecording() {
    if (g_isRecording) return false;
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = true;
    return true;
}

/**
 * Stop video recording.
 * Returns false if not currently recording.
 */
bool stopRecording() {
    if (!g_isRecording) return false;
    sendCameraControl(ACTION_POWER_BTN);
    g_isRecording = false;
    return true;
}

/**
 * Toggle recording state.
 * Starts if idle, stops if recording.
 */
void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
}

/**
 * Cycle to the next camera mode (video standby → QR/settings → video standby).
 * Resets recorded state tracking since mode change invalidates it.
 */
void changeMode() {
    sendCameraControl(ACTION_CHANGE_MODE, MODE_CHANGE_DELAY);
    g_isRecording = false;
}

/**
 * Returns true if the camera is ready to accept commands.
 */
bool isCameraReady() {
    return g_cameraReady;
}

/**
 * Returns true if the camera is currently recording.
 * Note: this is tracked locally — there is no way to query recording state
 * from the camera over UART.
 */
bool isRecording() {
    return g_isRecording;
}

// ── Initialisation ────────────────────────────────────────────────────────────

/**
 * Probe the camera and populate feature flags.
 * Call once from setup() after a 3-second boot delay.
 * Returns true if the camera was found and responded correctly.
 */
bool initCamera() {
    for (uint8_t i = 0; i < INIT_MAX_RETRIES; i++) {
        if (getDeviceInfo()) {
            g_cameraReady = true;
            g_isRecording = false;
            return true;
        }
        delay(INIT_RETRY_DELAY_MS);
    }
    return false;
}

// ── Example usage in setup/loop ───────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);

    // Camera needs ~3s to boot before accepting UART commands
    delay(3000);

    if (initCamera()) {
        Serial.printf("RunCam ready. Protocol v%u  Features=0x%04X\n",
                      g_protocolVer, g_cameraFeatures);
    } else {
        Serial.println("RunCam not found — check wiring and power supply");
    }
}

void loop() {
    // Example: drive via Serial monitor
    if (Serial.available()) {
        switch ((char)Serial.read()) {
            case 's': startRecording();  break;
            case 'x': stopRecording();   break;
            case 't': toggleRecording(); break;
            case 'm': changeMode();      break;
            case 'i':
                if (getDeviceInfo())
                    Serial.printf("Camera alive. v%u  0x%04X  recording=%s\n",
                                  g_protocolVer, g_cameraFeatures,
                                  g_isRecording ? "yes" : "no");
                else
                    Serial.println("No response");
                break;
            case 'r':
                Serial.println(recoverCamera() ? "Recovered" : "Recovery failed");
                break;
        }
    }
}