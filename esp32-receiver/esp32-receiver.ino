/*
  Project: USB HID Gamepad on ESP32-S3 from UART data
  Author: Camilo Herrera
  Description:
    - Exposes a USB HID Gamepad device (ESP32-S3 has native USB support).
    - Receives binary packets over UART (Serial1) at 250000 baud.
    - Validates each packet using CRC-8 (polynomial 0x31).
    - Filters and calibrates two axes (X, Y) and maps them to gamepad ranges.
    - Handles 32 buttons, sending updates to the host only when they change.
    - Runs on a dedicated FreeRTOS task for low latency and to avoid the watchdog reset.

  Packet format (10 bytes):
    [0] 0xAA               // Start byte (checked using peek)
    [1] X_H                // X high byte (bits 15..8)
    [2] X_L                // X low byte  (bits 7..0)
    [3] Y_H                // Y high byte
    [4] Y_L                // Y low byte
    [5] B3                 // Buttons [31:24]
    [6] B2                 // Buttons [23:16]
    [7] B1                 // Buttons [15:8]
    [8] B0                 // Buttons [7:0]
    [9] CRC8               // CRC-8 over bytes [1..8] (polynomial 0x31)

  Notes:
    - The axes are filtered using EMA (alpha = 0.25) to reduce noise while keeping responsiveness.
    - A deadzone is applied around the center to avoid jitter.
    - The task clears the RX buffer if there is backlog to reduce latency (“lag killer”).
*/

#include "USB.h"
#include "USBHIDGamepad.h"

// USB HID gamepad instance
USBHIDGamepad Gamepad;

// ---------- UART ----------
#define RX_PIN 1          // GPIO used as Serial1 RX (receive-only)
#define BAUD_RATE 250000  // High UART speed for low latency
#define PACKET_SIZE 10    // Fixed expected packet size

// ---------- CALIBRATION (adjust to your hardware) ----------
// Raw minimum, maximum and center values (from ADC/remote readings) for each axis
const int X_MIN = 1611, X_MAX = 2033, X_CENTER = 1833; 
const int Y_MIN = 2221, Y_MAX = 2641, Y_CENTER = 2387;

// ---------- FILTER & LATENCY ----------
// alpha: EMA filtering factor (higher = more responsive, less smoothing)
// deadzone: range around center where 0 is returned (removes jitter)
const float alpha = 0.25;
const int deadzone = 12;

// Filter state variables (initialized at center)
float fX = X_CENTER, fY = Y_CENTER;

/**
 * Calculates CRC-8 (polynomial 0x31) over a block of data.
 * @param data Pointer to the buffer to validate.
 * @param len  Length of data in bytes.
 * @return Calculated CRC (uint8_t).
 *
 * Implements CRC-8/SMBus: init=0x00, poly=0x31, refin/refout=false.
 * Used to validate bytes [1..8] of the packet.
 */
uint8_t getCRC8(uint8_t *data, byte len) {
  uint8_t crc = 0x00;
  for (byte b = 0; b < len; b++) {
    crc ^= data[b];
    for (byte i = 8; i > 0; i--) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else crc <<= 1;
    }
  }
  return crc;
}

/**
 * @brief Applies clamping, deadzone, and mapping to a filtered axis value for HID output.
 *
 * Process:
 *   1) Clamp the value to [minV, maxV].
 *   2) If near the center (deadzone), return 0.
 *   3) Linearly map to [-127..0] or [0..127] depending on which side of center.
 *   4) Final multiplier (here = 1) in case Windows needs earlier full-scale values.
 *   5) Safety clamp at ±32767 (prevents overflow if multiplier changes).
 *
 * @param filteredVal Filtered axis value (float).
 * @param minV Calibrated minimum value.
 * @param cenV Calibrated center value.
 * @param maxV Calibrated maximum value.
 * @return HID-ready integer axis value (int32_t).
 */
int32_t getCleanHID(float filteredVal, int minV, int cenV, int maxV) {
  // 1) Hard clamp to calibrated limits
  int32_t raw = (int32_t)filteredVal;
  if (raw < minV) raw = minV;
  if (raw > maxV) raw = maxV;
  
  // 2) Center deadzone
  if (abs(raw - cenV) < deadzone) return 0;

  // 3) 8-bit mapping (avoid overflow using int32_t)
  int32_t cleanStep;
  if (raw < cenV) {
    cleanStep = map(raw, minV, cenV, -127, 0);
  } else {
    cleanStep = map(raw, cenV, maxV, 0, 127);
  }

  // 4) Multiplier: adjust here if Windows doesn’t reach 100% with physical max
  int32_t finalHID = cleanStep * 1;

  // 5) Safety clamp (future-proof for 16-bit HID or higher multipliers)
  if (finalHID > 32767) finalHID = 32767;
  if (finalHID < -32767) finalHID = -32767;

  return finalHID;
}

/**
 * @brief FreeRTOS task that continuously processes UART data and updates the HID Gamepad.
 *
 * Logic:
 *   - Waits until a full 10-byte packet is available.
 *   - Flush old bytes if there is backlog.
 *   - Looks for header 0xAA, then reads 10 bytes.
 *   - Verifies CRC over bytes [1..8] vs byte [9].
 *   - Extracts X/Y axes (16-bit), applies EMA and calibration → leftStick().
 *   - Builds a 32‑bit button field and sends changes (press/release) when different.
 *   - Yields CPU if there is not enough data (prevents watchdog resets).
 */
void ProcessingLoop(void *pvParameters) {
  uint8_t packet[PACKET_SIZE];

  for (;;) {
    // Yield 1 ms if not enough bytes for a complete packet
    if (Serial1.available() < PACKET_SIZE) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // If the buffer is too full, discard old bytes
    // Keeps latency low by prioritizing the most recent packets
    while (Serial1.available() > PACKET_SIZE * 2) Serial1.read();

    // Check header byte without consuming it
    if (Serial1.peek() == 0xAA) {
      // Read the complete 10-byte packet
      Serial1.readBytes(packet, PACKET_SIZE);

      // Validate CRC-8 over bytes [1..8]; compare to packet[9]
      if (getCRC8(&packet[1], 8) == packet[9]) {
        // --- Axes: X and Y as 16-bit values ---
        uint16_t xRaw = (packet[1] << 8) | packet[2];
        uint16_t yRaw = (packet[3] << 8) | packet[4];

        // EMA filter for smoothing while keeping responsiveness
        fX = (alpha * (float)xRaw) + ((1.0f - alpha) * fX);
        fY = (alpha * (float)yRaw) + ((1.0f - alpha) * fY);

        // Apply calibration, deadzone, mapping, and send to left stick
        Gamepad.leftStick(
          getCleanHID(fX, X_MIN, X_CENTER, X_MAX), 
          getCleanHID(fY, Y_MIN, Y_CENTER, Y_MAX)
        );

        // --- Buttons: 32 buttons stored across 4 bytes (B3..B0) ---
        uint32_t btns = ((uint32_t)packet[5] << 24) |
                        ((uint32_t)packet[6] << 16) |
                        ((uint32_t)packet[7] << 8)  |
                        (uint32_t)packet[8];

        // Only send button events when their state changes
        static uint32_t lastBtns = 0;
        if (btns != lastBtns) {
          for (int i = 0; i < 32; i++) {
            uint32_t mask = (1UL << i);
            bool changed = ((btns & mask) != (lastBtns & mask));
            if (changed) {
              if (btns & mask) Gamepad.pressButton(i + 1);   // HID buttons 1..32
              else             Gamepad.releaseButton(i + 1);
            }
          }
          lastBtns = btns;
        }
      }
    } else {
      // If the first byte isn't the header, discard one byte to re-synchronize
      Serial1.read();
    }
  }
}

/**
 * @brief Device initialization.
 * - Initializes the HID Gamepad and USB stack.
 * - Configures UART Serial1 (RX only) at 250000 baud with a large RX buffer.
 * - Creates the processing task on core 0.
 */
void setup() {
  Gamepad.begin();                           // Initialize HID interface
  USB.begin();                               // Enable native USB for ESP32-S3
  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, -1); // UART1: RX on GPIO1, no TX
  Serial1.setRxBufferSize(1024);             // Increase buffer for burst data

  // Create the ProcessingLoop task on core 0 (4096 stack, priority 1)
  xTaskCreatePinnedToCore(ProcessingLoop, "ProcTask", 4096, NULL, 1, NULL, 0);
}

/**
 * @brief loop() is unused
 * All logic runs inside the FreeRTOS task.
 */
void loop() { 
  vTaskDelete(NULL); 
}