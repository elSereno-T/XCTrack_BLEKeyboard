/**
 * RunCam Thumb Pro W — ESP32-C3 UART Controller
 * ================================================
 * Controls start/stop recording via the RunCam Device Protocol.
 * Also supports setting camera date/time (SETTINGID_DISP_CAMERA_TIME).
 *
 * Wiring (ESP32-C3 ↔ RunCam Thumb Pro W 1.25mm 4P connector):
 *   RunCam TX  →  ESP32-C3 GPIO20 (UART1 RX)
 *   RunCam RX  →  ESP32-C3 GPIO21 (UART1 TX)
 *   RunCam GND →  ESP32-C3 GND
 *   RunCam PWR →  5V (NOT from ESP32 3.3V rail — use a proper 5V BEC)
 *
 * Baud rate: 115200
 *
 * Protocol notes for the Thumb Pro W:
 *   - Supports GET_DEVICE_INFO (0x00) for feature detection
 *   - Supports CAMERA_CONTROL (0x01) for start/stop recording
 *   - Uses SIMULATE_POWER_BTN (0x01) as a toggle — one press starts,
 *     next press stops. The camera does NOT send an ACK for 0x01,
 *     so we track state locally.
 *   - WRITE_SETTING (0x13) with setting ID 6 sets date/time as a string.
 *   - The camera does NOT support the 5-key OSD cable protocol (0x02-0x04).
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>
#include <cstdio>
#include <time.h>

// ── Pin & UART config ────────────────────────────────────────────────────────
#define RUNCAM_SERIAL     Serial1
#define RUNCAM_BAUD       115200
#define RUNCAM_RX_PIN     RX
#define RUNCAM_TX_PIN     TX

#define RUNCAM_POWER_PIN  D1
#define RUNCAM_EN_PIN     D2
#define RUNCAM_VOLT_PIN   A0

// ── Protocol constants ───────────────────────────────────────────────────────
static const uint8_t RC_HEADER = 0xCC;

// Command IDs
static const uint8_t CMD_GET_DEVICE_INFO   = 0x00;
static const uint8_t CMD_CAMERA_CONTROL    = 0x01;
static const uint8_t CMD_WRITE_SETTING     = 0x13;

// Camera control action IDs
static const uint8_t ACTION_SIMULATE_WIFI  = 0x00;
static const uint8_t ACTION_SIMULATE_POWER = 0x01;  // toggle recording on Thumb Pro
static const uint8_t ACTION_CHANGE_MODE    = 0x02;
static const uint8_t ACTION_START_REC      = 0x03;
static const uint8_t ACTION_STOP_REC       = 0x04;

// Feature flags (from GET_DEVICE_INFO response, uint16_t)
static const uint16_t FEAT_POWER_BTN       = (1 << 0);
static const uint16_t FEAT_WIFI_BTN        = (1 << 1);
static const uint16_t FEAT_CHANGE_MODE     = (1 << 2);
static const uint16_t FEAT_5KEY_OSD        = (1 << 3);
static const uint16_t FEAT_SETTINGS_ACCESS = (1 << 4);
static const uint16_t FEAT_DISPLAYPORT     = (1 << 5);
static const uint16_t FEAT_START_REC       = (1 << 6);
static const uint16_t FEAT_STOP_REC        = (1 << 7);

// Reserved setting IDs
static const uint8_t SETTING_CAMERA_TIME   = 6;   // STRING, Read & Write

// ── Timeouts ─────────────────────────────────────────────────────────────────
static const uint32_t RESPONSE_TIMEOUT_MS  = 500;
static const uint32_t INIT_RETRY_INTERVAL  = 1000;
static const uint8_t  INIT_MAX_RETRIES     = 10;

// ── State ────────────────────────────────────────────────────────────────────
static bool     g_cameraReady    = false;
static bool     g_isRecording    = false;
static uint16_t g_cameraFeatures = 0;
static uint8_t  g_protocolVer    = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
uint8_t crc8(const uint8_t *data, size_t len);
void    sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen);
size_t  receivePacket(uint8_t *buf, size_t maxLen);
bool    validateResponse(const uint8_t *buf, size_t len);
bool    getDeviceInfo();
void    sendCameraControl(uint8_t action);
bool    startRecording();
bool    stopRecording();
void    toggleRecording();
bool    setCameraTime(const char *timeStr);
bool    setCameraTimeFromTm(const struct tm *t);
bool    initCamera();

// ── CRC-8 (poly 0xD5) ────────────────────────────────────────────────────────
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
        }
    }
    return crc;
}

// ── Low-level send ────────────────────────────────────────────────────────────
void sendPacket(uint8_t cmd, const uint8_t *payload, size_t payloadLen) {
    size_t totalLen = 2 + payloadLen;   // header + cmd + payload (no crc yet)
    uint8_t buf[totalLen + 1];          // +1 for crc
    buf[0] = RC_HEADER;
    buf[1] = cmd;
    for (size_t i = 0; i < payloadLen; i++) {
        buf[2 + i] = payload[i];
    }
    buf[totalLen] = crc8(buf, totalLen);

    RUNCAM_SERIAL.write(buf, totalLen + 1);
    RUNCAM_SERIAL.flush();

    Serial.printf("[TX] cmd=0x%02X payload=%u bytes  crc=0x%02X\n",
                  cmd, (unsigned)payloadLen, buf[totalLen]);
}

// ── Low-level receive ─────────────────────────────────────────────────────────
size_t receivePacket(uint8_t *buf, size_t maxLen) {
    size_t count = 0;
    uint32_t deadline = millis() + RESPONSE_TIMEOUT_MS;

    while (millis() < deadline && count < maxLen) {
        if (RUNCAM_SERIAL.available()) {
            buf[count++] = (uint8_t)RUNCAM_SERIAL.read();
            deadline = millis() + 50;  // extend after each byte
        }
    }

    if (count > 0) {
        Serial.printf("[RX] %u bytes: ", (unsigned)count);
        for (size_t i = 0; i < count; i++) Serial.printf("0x%02X ", buf[i]);
        Serial.println();
    }
    return count;
}

bool validateResponse(const uint8_t *buf, size_t len) {
    if (len < 2) {
        Serial.println("[ERR] Response too short");
        return false;
    }
    if (buf[0] != RC_HEADER) {
        Serial.printf("[ERR] Bad header: 0x%02X (expected 0xCC)\n", buf[0]);
        return false;
    }
    uint8_t expected = crc8(buf, len - 1);
    if (buf[len - 1] != expected) {
        Serial.printf("[ERR] CRC mismatch: got 0x%02X expected 0x%02X\n",
                      buf[len - 1], expected);
        return false;
    }
    return true;
}

// ── GET_DEVICE_INFO (0x00) ────────────────────────────────────────────────────
bool getDeviceInfo() {
    Serial.println("[INFO] Sending GET_DEVICE_INFO...");
    sendPacket(CMD_GET_DEVICE_INFO, nullptr, 0);

    uint8_t resp[5];
    size_t n = receivePacket(resp, sizeof(resp));

    if (n < 5 || !validateResponse(resp, n)) {
        Serial.println("[WARN] GET_DEVICE_INFO: no valid response");
        return false;
    }

    g_protocolVer    = resp[1];
    g_cameraFeatures = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);

    Serial.printf("[INFO] Camera ready! Protocol v%d  Features=0x%04X\n",
                  g_protocolVer, g_cameraFeatures);

    if (g_cameraFeatures & FEAT_POWER_BTN)       Serial.println("  + Power button simulation");
    if (g_cameraFeatures & FEAT_WIFI_BTN)        Serial.println("  + WiFi button simulation");
    if (g_cameraFeatures & FEAT_CHANGE_MODE)     Serial.println("  + Change mode");
    if (g_cameraFeatures & FEAT_5KEY_OSD)        Serial.println("  + 5-key OSD");
    if (g_cameraFeatures & FEAT_SETTINGS_ACCESS) Serial.println("  + Settings access");
    if (g_cameraFeatures & FEAT_DISPLAYPORT)     Serial.println("  + DisplayPort");
    if (g_cameraFeatures & FEAT_START_REC)       Serial.println("  + Start recording");
    if (g_cameraFeatures & FEAT_STOP_REC)        Serial.println("  + Stop recording");

    return true;
}

// ── CAMERA_CONTROL (0x01) ─────────────────────────────────────────────────────
void sendCameraControl(uint8_t action) {
    sendPacket(CMD_CAMERA_CONTROL, &action, 1);
    // No ACK expected from Thumb Pro W for control commands
    delay(100);
    while (RUNCAM_SERIAL.available()) RUNCAM_SERIAL.read();  // drain
}

bool startRecording() {
    if (g_isRecording) {
        Serial.println("[WARN] Already recording — ignoring startRecording()");
        return false;
    }
    Serial.println("[INFO] Starting recording...");
    if (g_cameraFeatures & FEAT_START_REC) {
        sendCameraControl(ACTION_START_REC);
    } else {
        sendCameraControl(ACTION_SIMULATE_POWER);
    }
    g_isRecording = true;
    Serial.println("[INFO] Recording STARTED (state tracked locally)");
    return true;
}

bool stopRecording() {
    if (!g_isRecording) {
        Serial.println("[WARN] Not recording — ignoring stopRecording()");
        return false;
    }
    Serial.println("[INFO] Stopping recording...");
    if (g_cameraFeatures & FEAT_STOP_REC) {
        sendCameraControl(ACTION_STOP_REC);
    } else {
        sendCameraControl(ACTION_SIMULATE_POWER);
    }
    g_isRecording = false;
    Serial.println("[INFO] Recording STOPPED (state tracked locally)");
    return true;
}

void toggleRecording() {
    if (g_isRecording) stopRecording(); else startRecording();
}

// ── WRITE_SETTING (0x13) — date/time ─────────────────────────────────────────
bool setCameraTime(const char *timeStr) {
    if (!(g_cameraFeatures & FEAT_SETTINGS_ACCESS)) {
        Serial.println("[WARN] SETTINGS_ACCESS not advertised — trying anyway");
    }

    size_t strLen    = strlen(timeStr) + 1;  // include null terminator
    size_t payloadLen = 1 + strLen;          // setting ID + string
    uint8_t payload[payloadLen];
    payload[0] = SETTING_CAMERA_TIME;
    memcpy(payload + 1, timeStr, strLen);

    Serial.printf("[INFO] Setting camera time to: %s\n", timeStr);
    sendPacket(CMD_WRITE_SETTING, payload, payloadLen);

    // Response: [0xCC | result_code | update_menu | crc8] = 4 bytes
    uint8_t resp[4];
    size_t n = receivePacket(resp, sizeof(resp));

    if (n < 4 || !validateResponse(resp, n)) {
        Serial.println("[WARN] setCameraTime: no valid response (may still have worked)");
        return false;
    }

    if (resp[1] == 0) {
        Serial.println("[INFO] Camera time set successfully!");
        return true;
    }
    Serial.printf("[ERR] setCameraTime failed, result code: 0x%02X\n", resp[1]);
    return false;
}

bool setCameraTimeFromTm(const struct tm *t) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
    return setCameraTime(buf);
}

// ── Initialisation ────────────────────────────────────────────────────────────
bool initCamera() {
    Serial.println("[INFO] Initialising RunCam Thumb Pro W...");
    for (uint8_t attempt = 1; attempt <= INIT_MAX_RETRIES; attempt++) {
        Serial.printf("[INFO] Probe attempt %d/%d\n", attempt, INIT_MAX_RETRIES);
        if (getDeviceInfo()) {
            g_cameraReady = true;
            return true;
        }
        delay(INIT_RETRY_INTERVAL);
    }
    Serial.println("[ERR] Camera not found after all retries.");
    return false;
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== RunCam Thumb Pro W controller ===");

    RUNCAM_SERIAL.begin(RUNCAM_BAUD, SERIAL_8N1, RUNCAM_RX_PIN, RUNCAM_TX_PIN);

    Serial.println("[INFO] Waiting 3s for camera to boot...");
    delay(3000);

    if (!initCamera()) {
        Serial.println("[ERR] Running without camera — commands will be sent blind.");
    }

    // Uncomment to sync time after init (fill in real values or use NTP/RTC):
    // struct tm t = {};
    // t.tm_year = 2025 - 1900;
    // t.tm_mon  = 2;   // March (0-indexed)
    // t.tm_mday = 19;
    // t.tm_hour = 14;
    // t.tm_min  = 30;
    // t.tm_sec  = 0;
    // setCameraTimeFromTm(&t);

    Serial.println("[INFO] Ready. Serial commands:");
    Serial.println("  's' -> start recording");
    Serial.println("  'x' -> stop  recording");
    Serial.println("  't' -> toggle recording");
    Serial.println("  'i' -> query device info");
    Serial.println("  'p' -> simulate Power button");
    Serial.println("  'd' -> set camera time to 2025-03-19 12:00:00");
}

void loop() {
    if (Serial.available()) {
        char c = (char)Serial.read();
        switch (c) {
            case 's': startRecording();                     break;
            case 'x': stopRecording();                      break;
            case 't': toggleRecording();                    break;
            case 'i': getDeviceInfo();                      break;
            case 'd': setCameraTime("2025-03-19 12:00:00"); break;
            case 'p': sendCameraControl(ACTION_SIMULATE_POWER); break;
            default:  break;
        }
    }
}