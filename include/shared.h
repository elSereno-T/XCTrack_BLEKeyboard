
#ifndef SHARED_H
#define SHARED_H

#include <Arduino.h>

#pragma once
// Standard BLE Battery Service (Bluetooth SIG)
#define BATTERY_SERVICE_UUID    "0x180F"
#define BATTERY_CHAR_UUID       "0x2A19"   // "Battery Level" characteristic

// ── Custom GATT service UUID ──────────────────────────────────────────────────
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890AB"

// Host writes a single byte here to send commands
#define CMD_CHAR_UUID       "12345678-1234-1234-1234-1234567890AC"

// Client sends a single byte notify here to ACK
#define ACK_CHAR_UUID       "12345678-1234-1234-1234-1234567890AD"

// Client advertises this name prefix so the host can identify it during scan
#define CLIENT_NAME  "RunCam"

// ── Command bytes ─────────────────────────────────────────────────────────────
#define CMD_START           0x01
#define CMD_STOP            0x02

// ── ACK bytes ─────────────────────────────────────────────────────────────────
#define ACK_STARTED         0x11
#define ACK_STOPPED         0x22

// ── Tuning ────────────────────────────────────────────────────────────────────
#define DISCOVERY_TIMEOUT_MS  60000
#define ACK_TIMEOUT_MS        3000
#define MAX_CLIENTS           5


#define BAUD 9600
#define INITIAL_DELAY 2000

#define Max_TX_Power_db 9

typedef enum {CAMERA_READY, CAMERA_BOOTING, CAMERA_RECORDING, CAMERA_SHUTDOWN, CAMERA_CONNECTING, CAMERA_DISCONNECTED, CAMERA_SCANNING, CAMERA_STOPPING, CAMERA_OFF } CameraState ;
const String CameraStateString[] = {"READY", "BOOTING", "RECORDING", "SHUTDOWN", "CONNECTING","DISCONNECTED", "SCANNING", "STOPPING", "OFF"};
CameraState camState = CAMERA_OFF;

#endif