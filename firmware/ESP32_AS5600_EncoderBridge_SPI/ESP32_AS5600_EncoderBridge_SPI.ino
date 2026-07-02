/*
 * ESP32_AS5600_EncoderBridge_SPI
 *
 * Emulates a MagAlpha MA732 SPI absolute encoder using an AS5600 read over I2C,
 * for the ODrive's native ENCODER_MODE_SPI_ABS_MA732 driver. Unlike the
 * quadrature-emulation sketch (../ESP32_AS5600_EncoderBridge/), this gives the
 * ODrive a TRUE absolute encoder: no per-boot re-homing spin, phase offset
 * calibration should persist for real, matching how the project's other 3
 * wheels already work (real SPI absolute MagAlpha-family encoder, confirmed
 * via their saved configs: spi_encoder0 enabled, hall/incremental disabled).
 * See ../../ODriveV3_6_Test.md for the full story of why this replaces the
 * quadrature approach.
 *
 * STATUS: SHELVED -- NOT the production setup. The quadrature bridge
 * (../ESP32_AS5600_EncoderBridge/) is what node 33 actually runs.
 *
 * What was proven on real hardware before shelving:
 *   - SPI link solid: byte order and wiring correct first try, ~8 kHz ODrive
 *     polling, spi_error_rate=0.0 throughout.
 *   - THE GOAL WORKED: pre_calibrated persisted across a real ODrive reboot
 *     with zero re-calibration -- boots straight into closed loop.
 * Why it was shelved anyway: commutation was UNSTABLE under actual motion --
 * commanding velocity produced huge current with little/wrong motion,
 * escalating to SPINOUT_DETECTED + CURRENT_LIMIT_VIOLATION trips. Suspected
 * root cause: the bridge's I2C-read -> SPI-serve latency acts as a small
 * constant angle error; the offset calibration (slow scan) absorbs it
 * consistently, but at real operating speed the commutation phase reference
 * is wrong enough to destabilize FOC. Offset cal reproduced IDENTICAL
 * phase_offset_float across runs -- consistency masked the systematic error.
 * If revisited: cut latency (faster I2C polling / predictive extrapolation
 * of the served angle) before trusting it with motion again.
 *
 * Two real issues found and FIXED during bring-up (fixes are in this file,
 * both empirically confirmed with data -- keep them if reviving this):
 *   1. A single glitched I2C read landing straight in g_latest_angle looked to
 *      the ODrive exactly like a real momentary velocity spike, causing actual
 *      Iq current spikes (up to 13.66A) on a STATIONARY motor. Fixed with a
 *      wraparound-aware plausibility filter (see MAX_PLAUSIBLE_JUMP) that
 *      rejects single-cycle jumps no real motion could produce.
 *   2. SPI polling ran at ~8000 txn/s but I2C only refreshed at ~4200 reads/s
 *      -- confirmed via the i2c_reads counter -- so roughly every other SPI
 *      transaction served a stale value. Mitigated by skipping the redundant
 *      register-pointer write on every read (the AS5600 keeps its internal
 *      pointer across transactions) -- roughly halves I2C traffic per read.
 *      Re-verify the reads/sec ratio after reflashing; if still not keeping
 *      up, the remaining gap is Wire library overhead, not the I2C bus itself.
 *
 * Protocol (from ODrive fw-v0.5.6 encoder.cpp, MODE_SPI_ABS_MA732):
 *   - 16-bit SPI frames, SPI mode 3 (CPOL=1, CPHA=1).
 *   - ODrive (master) clocks out a dummy 0x0000 and reads back 16 bits; bits
 *     [15:2] are taken as the 14-bit angle, bits [1:0] are ignored. No
 *     checksum/parity needed for this mode (unlike AMS/CUI modes).
 *
 * Wiring:
 *   AS5600  VCC->3V3  GND->GND  SDA->GPIO21  SCL->GPIO22   (same as the quadrature sketch)
 *   ODrive  SCK/MISO/MOSI: hardwired on v3.x -- locate physically, see STATUS above.
 *           nCS: any spare GPIO set to GPIO_MODE_DIGITAL, referenced via
 *                encoder.config.abs_spi_cs_gpio_pin.
 *   ESP32 (this sketch, default VSPI pins):
 *           SCK  -> GPIO18
 *           MISO -> GPIO19   (this is OUR output -- the angle data)
 *           MOSI -> GPIO23   (ODrive's dummy byte, ignored)
 *           CS   -> GPIO5    (must match whatever GPIO you set abs_spi_cs_gpio_pin to)
 *           GND  <-> ODrive GND (common ground, required)
 *
 * ODrive-side config:
 *   odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_MA732
 *   odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = <chosen GPIO>   # set that GPIO's
 *       odrv0.config.gpioN_mode = GPIO_MODE_DIGITAL first
 *   odrv0.axis0.encoder.config.cpr = 16384   # 14-bit
 *   -- offset calibration should now only need to run ONCE ever; pre_calibrated
 *      should persist across reboots for real, since this is a true absolute
 *      encoder from the ODrive's point of view.
 *
 * Implementation:
 *   - Uses ESP-IDF's driver/spi_slave.h directly -- Arduino's SPI.h is master-only.
 *   - A dedicated FreeRTOS task (pinned to core 0) continuously polls the AS5600
 *     over I2C into a shared g_latest_angle. The main Arduino loop() (core 1)
 *     continuously arms one SPI slave transaction at a time with whatever's
 *     freshest in g_latest_angle, so SPI response latency is never blocked
 *     behind an I2C transaction.
 *   - Stateless by design: every transaction just reports "the angle right
 *     now" -- no running count, no unwrap, no aliasing risk (unlike the
 *     quadrature sketch, which had all three).
 *   - WiFi/Bluetooth disabled at boot, same rationale as the quadrature sketch:
 *     the SPI slave's response timing needs to stay jitter-free.
 */

#include <Wire.h>
#include <WiFi.h>
#include "esp32-hal-bt.h"
#include "driver/spi_slave.h"

// ---------- pin assignment ----------
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const int SPI_SCK_PIN = 18;
static const int SPI_MISO_PIN = 19;
static const int SPI_MOSI_PIN = 23;
static const int SPI_CS_PIN = 5;
static const spi_host_device_t SPI_SLAVE_HOST = VSPI_HOST;

// ---------- AS5600 ----------
static const uint8_t AS5600_ADDR = 0x36;
static const uint8_t AS5600_REG_RAW_ANGLE = 0x0C;  // 2 bytes: [11:8] then [7:0]
static const uint32_t I2C_CLOCK_HZ = 400000;        // AS5600 Fast-mode max

static volatile uint16_t g_latest_angle = 0;  // 0..4095, guarded by g_angle_mux
static portMUX_TYPE g_angle_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_i2c_error_count = 0;
static volatile uint32_t g_i2c_read_count = 0;
static volatile uint32_t g_i2c_rejected_count = 0;  // plausibility filter, see below
static volatile uint32_t g_spi_txn_count = 0;

// A single glitched I2C read landing straight in g_latest_angle looks to the
// ODrive exactly like a real momentary velocity spike (confirmed empirically:
// a single stray sample caused a real Iq current spike on the real motor).
// This rejects any read that jumps further than physically plausible for one
// read cycle. 300 counts/cycle at ~5-8kHz read rate is already >150 rev/s --
// far beyond anything this motor can actually do -- so this only catches
// glitches, never real motion.
static const int32_t MAX_PLAUSIBLE_JUMP = 300;  // counts, wraparound-aware

int32_t wrappedDelta(int32_t from, int32_t to) {
  int32_t d = to - from;
  if (d > 2048) d -= 4096;
  else if (d < -2048) d += 4096;
  return d;
}

// The AS5600 keeps its internal register pointer across separate transactions
// (only a fresh write changes it), so after the first read we skip re-sending
// the register address and just re-issue requestFrom() -- this alone roughly
// halves the I2C traffic per read (2 phases instead of 4).
static bool g_pointer_set = false;

bool setAS5600Pointer() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_RAW_ANGLE);
  return Wire.endTransmission(false) == 0;  // repeated start, keep bus held
}

bool readAS5600Raw(uint16_t &out) {
  if (!g_pointer_set) {
    if (!setAS5600Pointer()) return false;
    g_pointer_set = true;
  }
  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) {
    g_pointer_set = false;  // bus may have desynced -- re-send the pointer next time
    return false;
  }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  out = (((uint16_t)hi << 8) | lo) & 0x0FFF;
  return true;
}

void i2cTask(void *arg) {
  uint16_t last_good = 0;
  bool have_last = false;
  for (;;) {
    uint16_t raw;
    if (readAS5600Raw(raw)) {
      bool accept = true;
      if (have_last && abs(wrappedDelta(last_good, raw)) > MAX_PLAUSIBLE_JUMP) {
        accept = false;
        g_i2c_rejected_count++;
      }
      if (accept) {
        last_good = raw;
        have_last = true;
        portENTER_CRITICAL(&g_angle_mux);
        g_latest_angle = raw;
        portEXIT_CRITICAL(&g_angle_mux);
        g_i2c_read_count++;
      }
    } else {
      g_i2c_error_count++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Same rationale as the quadrature sketch: kill the two biggest sources of
  // interrupt/timing jitter before anything else touches the SPI slave timing.
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(240);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

  // Prime with a real reading before the SPI slave starts answering, so the
  // first transactions don't report angle 0 out of reset.
  uint16_t raw0;
  while (!readAS5600Raw(raw0)) {
    Serial.println("waiting for AS5600...");
    delay(100);
  }
  g_latest_angle = raw0;

  xTaskCreatePinnedToCore(i2cTask, "i2c_task", 4096, NULL, 1, NULL, 0);

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = SPI_MOSI_PIN;
  buscfg.miso_io_num = SPI_MISO_PIN;
  buscfg.sclk_io_num = SPI_SCK_PIN;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.mode = 3;  // CPOL=1, CPHA=1 -- matches ODrive's MA732 driver config
  slvcfg.spics_io_num = SPI_CS_PIN;
  slvcfg.queue_size = 3;
  slvcfg.flags = 0;

  esp_err_t err = spi_slave_initialize(SPI_SLAVE_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    Serial.printf("spi_slave_initialize FAILED: %d\n", err);
    while (1) delay(1000);
  }

  Serial.printf("Started. raw0=%u  SPI mode 3, CS=GPIO%d SCK=GPIO%d MISO=GPIO%d MOSI=GPIO%d\n",
                raw0, SPI_CS_PIN, SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
}

void loop() {
  static uint8_t tx_buf[2];
  static uint8_t rx_buf[2];

  uint16_t angle;
  portENTER_CRITICAL(&g_angle_mux);
  angle = g_latest_angle;
  portEXIT_CRITICAL(&g_angle_mux);

  // angle in bits [15:2], bits [1:0] = 0 -- matches encoder.cpp's
  // `pos = (rawVal >> 2) & 0x3fff` decode on the ODrive side.
  uint16_t frame = (angle << 2) & 0xFFFC;
  tx_buf[0] = (frame >> 8) & 0xFF;  // MSB first -- see STATUS note if this looks wrong
  tx_buf[1] = frame & 0xFF;

  spi_slave_transaction_t t = {};
  t.length = 16;  // bits
  t.tx_buffer = tx_buf;
  t.rx_buffer = rx_buf;

  // Blocks until the ODrive clocks this transaction or the timeout lapses,
  // then we immediately loop back and arm the next one with fresher data.
  esp_err_t err = spi_slave_transmit(SPI_SLAVE_HOST, &t, pdMS_TO_TICKS(50));
  if (err == ESP_OK) {
    g_spi_txn_count++;
  }

  static uint32_t last_print = 0;
  uint32_t now = millis();
  if (now - last_print >= 200) {
    last_print = now;
    Serial.printf("angle=%4u frame=0x%04X spi_txns=%lu i2c_reads=%lu i2c_errs=%lu i2c_rejected=%lu\n",
                  angle, frame, (unsigned long)g_spi_txn_count,
                  (unsigned long)g_i2c_read_count, (unsigned long)g_i2c_error_count,
                  (unsigned long)g_i2c_rejected_count);
  }
}
