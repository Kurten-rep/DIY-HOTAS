#include <Wire.h>
#include <WiFi.h>
#include "esp_wifi.h"


/* ================================
 *           Pin Mapping
 * ================================
 * Shift-register (e.g., 74HC165 chain) to read up to 32 buttons:
 *  - PIN_PL : Parallel Load (/PL). Low pulse latches the current button states.
 *  - PIN_CP : Shift Clock. Rising edge shifts the next bit to Q7.
 *  - PIN_Q7 : Serial output from the last register in chain (MSB-first in this code).
 *
 * AS5600 I²C magnetic angle sensors (two units with same address):
 *  - Use two separate I²C buses to avoid address conflict (both at 0x36).
 */

const int PIN_PL = 13;
const int PIN_CP = 18;
const int PIN_Q7 = 19;
const uint8_t AS5600_ADDR = 0x36;

/* ================================
 *     Filtering & Timing State
 * ================================
 * lastX / lastY: hold the previous valid angle reading to enable anti-jump filtering.
 * MAX_JUMP: maximum allowed absolute difference between consecutive readings; larger
 *           jumps are treated as glitches and ignored. (Range is 0..4095 for AS5600)
 */
uint16_t lastX = 2048, lastY = 2048;
const uint16_t MAX_JUMP = 600;  // ~15% of full scale (4096)

/* FreeRTOS task handle for the high-priority loop that samples inputs and sends packets. */
TaskHandle_t FlightTaskHandle;

/* ===========================================================
 * CRC-8 (MSB-first, polynomial 0x31) — Non-reflected variant
 * ===========================================================
 * Computes an 8-bit CRC over 'len' bytes in 'data'.
 * - Initial value: 0x00
 * - Polynomial   : 0x31 (MSB-first, left-shift)
 *
 * Important: This is NOT the typical Dallas/Maxim reflected implementation.
 * Ensure your receiver computes the same variant or change this to the ref/LSB-first form.
 */
uint8_t getCRC8(uint8_t *data, byte len) {
  uint8_t crc = 0x00;
  for (byte b = 0; b < len; b++) {
    crc ^= data[b];
    for (byte i = 8; i > 0; i--) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;  // if MSB set, shift then XOR polynomial
      else crc <<= 1;                           // else just shift
    }
  }
  return crc;
}

/* ===========================
 *   Function Declarations
 * ===========================
 * readValidatedAngle(): Reads 2-byte angle from AS5600 and applies anti-jump filter.
 * readButtons():        Captures 32 button bits from 74HC165 chain (MSB-first).
 * FlightLoop():         Real-time task that builds and transmits the packet at 1 kHz.
 */
uint16_t readValidatedAngle(TwoWire &bus, uint16_t &lastVal);
uint32_t readButtons();
void FlightLoop(void *pvParameters);

void setup() {
  /* -------------------------------------------
   * Disable Bluetooth radio on ESP32 (Arduino-ESP32 Core 3.x)
   * to reduce latency and CPU jitter for a real-time task.
   * ------------------------------------------- */
  btStop();

  
  // Disable WiFi completely    
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();



  /* -------------------------------------------
   * UART2 initialization at 250000 baud, 8N1.
   * Pins: RX = GPIO16, TX = GPIO17.
   * This is the link used to stream the packed data frames.
   * ------------------------------------------- */
  Serial2.begin(250000, SERIAL_8N1, 16, 17);

  /* -------------------------------------------
   * I²C setup for two separate buses:
   *  - Wire  : SDA=21, SCL=22  (first AS5600)
   *  - Wire1 : SDA=32, SCL=33  (second AS5600)
   * Both run at 400 kHz (Fast mode).
   * Using two buses solves the single-address (0x36) limitation.
   * ------------------------------------------- */
  Wire.begin(21, 22);
  Wire1.begin(32, 33);
  Wire.setClock(400000);
  Wire1.setClock(400000);

  /* -------------------------------------------
   * Shift-register GPIO setup (74HC165 chain).
   * Ensure initial clock low and PL high (no latch).
   * ------------------------------------------- */
  pinMode(PIN_PL, OUTPUT);
  pinMode(PIN_CP, OUTPUT);
  pinMode(PIN_Q7, INPUT);
  digitalWrite(PIN_CP, LOW);
  digitalWrite(PIN_PL, HIGH);

  /* -------------------------------------------
   * Create high-priority FreeRTOS task on Core 1.
   * - Stack size: 4096 bytes
   * - Priority : 24 (very high; avoid blocking calls inside)
   * - Frequency: 1 kHz cycle (handled inside the task)
   * ------------------------------------------- */
  xTaskCreatePinnedToCore(FlightLoop, "FlightTask", 4096, NULL, 24, &FlightTaskHandle, 1);
}

/* ===========================================================
 * FlightLoop (FreeRTOS task)
 * ===========================================================
 * Runs at ~1 ms period using vTaskDelayUntil for precise timing.
 * Each cycle:
 *  1) Reads angle from AS5600 on Wire  and filters it.
 *  2) Reads angle from AS5600 on Wire1 and filters it.
 *  3) Reads 32 buttons via shift-register chain.
 *  4) Builds a 10-byte packet:
 *     [0] Header (0xAA)
 *     [1] X_HI
 *     [2] X_LO
 *     [3] Y_HI
 *     [4] Y_LO
 *     [5] Buttons[31:24]  (MSB first)
 *     [6] Buttons[23:16]
 *     [7] Buttons[15:8]
 *     [8] Buttons[7:0]    (LSB)
 *     [9] CRC8 over bytes [1..8]
 *  5) Sends the packet over UART2.
 */
void FlightLoop(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1 ms period (1 kHz)

  for (;;) {
    // Read both angle sensors with anti-jump protection.
    uint16_t x = readValidatedAngle(Wire,  lastX);
    uint16_t y = readValidatedAngle(Wire1, lastY);

    // Read 32 buttons into a 32-bit mask (bit 31 = first shifted bit).
    uint32_t btns = readButtons();

    // Build packet (10 bytes total).
    uint8_t packet[10];
    packet[0] = 0xAA;                 // Header (not included in CRC)
    packet[1] = (uint8_t)(x >> 8);    // X high byte
    packet[2] = (uint8_t)(x & 0xFF);  // X low byte
    packet[3] = (uint8_t)(y >> 8);    // Y high byte
    packet[4] = (uint8_t)(y & 0xFF);  // Y low byte
    packet[5] = (uint8_t)(btns >> 24);
    packet[6] = (uint8_t)(btns >> 16);
    packet[7] = (uint8_t)(btns >> 8);
    packet[8] = (uint8_t)(btns & 0xFF);

    // Compute CRC over the data section only: bytes [1..8]
    packet[9] = getCRC8(&packet[1], 8);

    // Transmit the packet atomically.
    Serial2.write(packet, 10);

    // Wait until next 1 ms tick (jitter-resistant).
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* ===========================================================
 * readValidatedAngle
 * ===========================================================
 * Reads the 16-bit ANGLE register (MSB at 0x0E, LSB at 0x0F) from AS5600.
 * The sensor provides 12-bit resolution (0..4095), packed into two bytes.
 * Anti-jump filter:
 *   - If the absolute difference to the last valid sample exceeds MAX_JUMP,
 *     the new reading is ignored to suppress spikes/glitches.
 *
 * Note: Because angles wrap around (0 <-> 4095), large jumps near wrap could be real.
 * A more advanced "circular" filter could be used:
 *   diff = (raw - lastVal + 4096) % 4096; if (diff > 2048) diff -= 4096; then test |diff|.
 */
uint16_t readValidatedAngle(TwoWire &bus, uint16_t &lastVal) {
  // Point to MSB register (0x0E) of the ANGLE register pair.
  bus.beginTransmission(AS5600_ADDR);
  bus.write(0x0E);
  if (bus.endTransmission() != 0) return lastVal;  // On bus error, keep last value.

  // Request two bytes (MSB then LSB).
  bus.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (bus.available() >= 2) {
    uint16_t raw = (bus.read() << 8) | bus.read();  // Combine bytes

    // Simple anti-jump check (non-circular).
    if (abs((int)raw - (int)lastVal) > MAX_JUMP) return lastVal;

    // Accept and store new value.
    lastVal = raw;
    return raw;
  }

  // If not enough bytes available, return last valid reading.
  return lastVal;
}

/* ===========================================================
 * readButtons
 * ===========================================================
 * Captures and shifts out 32 bits from a 74HC165 chain.
 * Procedure:
 *  1) Pull /PL low briefly to latch the current parallel inputs.
 *  2) Return /PL high (normal mode).
 *  3) For 32 cycles:
 *      - Read Q7 (MSB-first assumption in this code).
 *      - Pulse the clock to shift next bit.
 *
 * Bit ordering:
 *  - First bit read is placed into bit 31 of the 32-bit result.
 *  - Last bit read lands in bit 0. Ensure this matches your receiver mapping.
 */
uint32_t readButtons() {
  uint32_t data = 0;

  // Latch parallel inputs into internal register.
  digitalWrite(PIN_PL, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_PL, HIGH);
  delayMicroseconds(5);

  // Shift out 32 bits, MSB-first.
  for (int i = 0; i < 32; i++) {
    if (digitalRead(PIN_Q7)) data |= (1UL << (31 - i));  // Map first bit to MSB (bit 31)
    digitalWrite(PIN_CP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_CP, LOW);
    delayMicroseconds(5);
  }
  return data;
}

/* ===========================================================
 * loop (Arduino entry)
 * ===========================================================
 * Not used. The FreeRTOS task handles the real-time work on Core 1.
 * Delete this default task to free Core 0 cycles.
 */
void loop() { vTaskDelete(NULL); }
