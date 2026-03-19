
#ifndef SHARED_H
#define SHARED_H

#include <Arduino.h>


// ── Host GATT services (host acts as peripheral for these) ────────────────────
#define HOST_SERVICE_UUID           "4fc05c90-6287-4618-93f8-702ba6e440f5"
#define HOST_NMEA_CHAR_UUID         "dc1ccec6-30a8-4ebc-9210-c7531e7cb4e1"


#pragma once
// Standard BLE Battery Service (Bluetooth SIG)
#define BATTERY_SERVICE_UUID    "180F"
#define BATTERY_CHAR_UUID       "2A19"   // "Battery Level" characteristic

// ── Custom GATT service UUID ──────────────────────────────────────────────────
#define SERVICE_UUID        "9f710bb7-e2c1-4a70-8af2-aae4226b60c4"

// Host writes a single byte here to send commands
#define CMD_CHAR_UUID       "49d0ae2e-74c0-408e-bdad-01986a24b40f"

// Client sends a single byte notify here to ACK
#define ACK_CHAR_UUID       "57a0e065-c8bf-4164-a0d0-48d4858bbbff"



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
const String ShortCameraStateString[] = {"READY", "BOOT", "REC.", "STOP", "CONN.","DISCON", "SCAN", "STOP", "OFF"};
CameraState camState = CAMERA_OFF;

const char* HostMAC = "98:3d:ae:ab:a4:d6";

#endif