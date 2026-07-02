/*
 * ESP32_AS5600_EncoderBridge
 *
 * Bridges an AS5600 magnetic angle sensor (I2C-only, no native SPI/ABI output)
 * to a synthesized quadrature A/B + Z-index signal that the ODrive v3.6 can
 * consume directly via its native ENCODER_MODE_INCREMENTAL input -- no ODrive
 * firmware changes needed.
 *
 * Board: ESP32-WROOM-32. Timer setup below auto-selects between the Arduino-ESP32
 * core 3.x (ESP-IDF 5.x) and core 2.x legacy timer APIs at compile time, so this
 * builds either way -- check Boards Manager if you need to know which you have
 * (Arduino IDE version and ESP32 core/board-package version are separate things).
 *
 * Wiring:
 *   AS5600   VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22
 *   ODrive   A   <- GPIO25, B <- GPIO26, Z <- GPIO27, GND <-> ESP32 GND (common ground)
 *   (ESP32 GPIO is 3.3V logic -- matches the ODrive's STM32F405 inputs directly.)
 *
 * ODrive-side config (once wired):
 *   odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
 *   odrv0.axis1.encoder.config.cpr  = 4096      # matches AS5600's native 12-bit resolution
 *   odrv0.axis1.config.gpio12_mode  = ENC1      # A
 *   odrv0.axis1.config.gpio13_mode  = ENC1      # B
 *   odrv0.axis1.config.gpio14_mode  = ENC1      # Z (if using this pin for index)
 *   -- run AXIS_STATE_MOTOR_CALIBRATION + AXIS_STATE_ENCODER_OFFSET_CALIBRATION once,
 *      save pre_calibrated=True on both, then AXIS_STATE_ENCODER_INDEX_SEARCH on
 *      subsequent boots before AXIS_STATE_CLOSED_LOOP_CONTROL.
 *   -- if the wheel spins the "wrong" way relative to command sign, flip
 *      encoder.config.direction rather than re-wiring A/B.
 *
 * Design notes (see chat/README for full rationale):
 *   - WiFi and Bluetooth are disabled at boot -- they are the main source of
 *     interrupt jitter that would otherwise corrupt the quadrature timing this
 *     depends on for real closed-loop commutation.
 *   - A hardware timer ISR steps the emitted quadrature count toward the
 *     latest AS5600 reading by at most 1 count per tick, at TIMER_FREQ_HZ.
 *     This bounds the edge rate seen by the ODrive and smooths out the
 *     "bursty" arrival of new I2C readings instead of jumping instantly.
 *   - TIMER_FREQ_HZ (80 kHz) is set with >3x margin over the fastest edge
 *     rate this application ever needs (~24.6 kcounts/s at max wheel speed:
 *     40 wheel RPM * 9:1 reduction = 360 motor RPM = 6 rev/s * 4096 cpr).
 *   - The Z pulse is level-based (a small angular window around the raw
 *     zero-crossing), not edge/one-shot -- self-correcting in both spin
 *     directions, no extra state to desync.
 */

#include <Wire.h>
#include <WiFi.h>
#include "esp32-hal-bt.h"

// ---------- pin assignment ----------
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const int A_PIN = 25;
static const int B_PIN = 26;
static const int Z_PIN = 27;

// ---------- AS5600 ----------
static const uint8_t AS5600_ADDR = 0x36;
static const uint8_t AS5600_REG_RAW_ANGLE = 0x0C;   // 2 bytes: [11:8] then [7:0]
static const uint32_t I2C_CLOCK_HZ = 400000;         // AS5600 Fast-mode max

// ---------- quadrature synthesis ----------
static const int32_t CPR = 4096;                    // AS5600 native counts/rev == ODrive cpr
static const int32_t Z_WINDOW = 8;                  // +/- counts around raw==0 where Z is high
static const uint32_t TIMER_FREQ_HZ = 80000;         // ISR rate, see design notes above

// forward=00,01,11,10 (standard x4 quadrature sequence)
static const uint8_t QUAD_A[4] = {0, 1, 1, 0};
static const uint8_t QUAD_B[4] = {0, 0, 1, 1};

hw_timer_t *timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int32_t target_count = 0;   // written from loop() under critical section
volatile int32_t emitted_count = 0;  // owned by the ISR
volatile uint8_t quad_state = 0;     // owned by the ISR, = emitted_count & 3

int32_t accumulated = 0;             // unwrapped position, owned by loop()
int32_t last_raw = -1;
bool last_raw_valid = false;
volatile uint32_t i2c_error_count = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&mux);
  int32_t tgt = target_count;
  portEXIT_CRITICAL_ISR(&mux);

  if (emitted_count < tgt) {
    emitted_count++;
    quad_state = (quad_state + 1) & 3;
  } else if (emitted_count > tgt) {
    emitted_count--;
    quad_state = (quad_state + 3) & 3;  // -1 mod 4
  } else {
    return;  // already in sync, nothing to toggle
  }

  digitalWrite(A_PIN, QUAD_A[quad_state]);
  digitalWrite(B_PIN, QUAD_B[quad_state]);

  // CPR is a power of 2, so this AND gives the correct non-negative
  // modulo even for negative emitted_count (two's complement).
  uint32_t m = (uint32_t)emitted_count & (CPR - 1);
  bool z = (m < Z_WINDOW) || (m > (uint32_t)(CPR - Z_WINDOW));
  digitalWrite(Z_PIN, z ? HIGH : LOW);
}

bool readAS5600Raw(uint16_t &out) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_RAW_ANGLE);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start, keep bus held
  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  out = (((uint16_t)hi << 8) | lo) & 0x0FFF;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Kill the two biggest sources of interrupt/timing jitter before anything
  // else -- the quadrature ISR's regularity is what keeps the ODrive's
  // commutation reference clean.
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(240);

  pinMode(A_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  pinMode(Z_PIN, OUTPUT);
  digitalWrite(A_PIN, LOW);
  digitalWrite(B_PIN, LOW);
  digitalWrite(Z_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

  // Prime with a real reading before starting the timer so the ISR doesn't
  // ramp from 0 through a few thousand spurious counts on boot.
  uint16_t raw0;
  while (!readAS5600Raw(raw0)) {
    Serial.println("waiting for AS5600...");
    delay(100);
  }
  last_raw = raw0;
  last_raw_valid = true;
  accumulated = raw0;
  target_count = raw0;
  emitted_count = raw0;
  quad_state = (uint8_t)(raw0 & 3);
  digitalWrite(A_PIN, QUAD_A[quad_state]);
  digitalWrite(B_PIN, QUAD_B[quad_state]);

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Arduino-ESP32 core 3.x (ESP-IDF 5.x) timer API
  timer = timerBegin(TIMER_FREQ_HZ);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1, true, 0);  // fire every tick (1 / TIMER_FREQ_HZ seconds), auto-reload
#else
  // Arduino-ESP32 core 2.x legacy timer API. APB clock is 80 MHz; divider 100
  // -> 800 kHz tick rate; alarm every 10 ticks -> exactly TIMER_FREQ_HZ (80 kHz).
  timer = timerBegin(0, 100, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10, true);
  timerAlarmEnable(timer);
#endif

  Serial.printf("Started. raw0=%u cpr=%ld timer=%lu Hz\n", raw0, (long)CPR, (unsigned long)TIMER_FREQ_HZ);
}

void loop() {
  uint16_t raw;
  if (readAS5600Raw(raw)) {
    int32_t delta = (int32_t)raw - last_raw;
    if (delta > CPR / 2) delta -= CPR;
    else if (delta < -CPR / 2) delta += CPR;
    accumulated += delta;
    last_raw = raw;

    portENTER_CRITICAL(&mux);
    target_count = accumulated;
    portEXIT_CRITICAL(&mux);
  } else {
    i2c_error_count++;
  }

  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= 200) {
    last_print = now;
    int32_t lag = accumulated - emitted_count;
    Serial.printf("raw=%4d accum=%8ld emitted=%8ld lag=%5ld i2c_errs=%lu\n",
                  last_raw, (long)accumulated, (long)emitted_count, (long)lag,
                  (unsigned long)i2c_error_count);
  }
}
