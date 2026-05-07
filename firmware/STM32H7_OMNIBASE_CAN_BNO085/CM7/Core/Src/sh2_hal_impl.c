/*
 * sh2_hal_impl.c
 *
 * SH2 HAL implementation for STM32H7 over I2C1, for use inside a FreeRTOS task.
 * Implements the sh2_Hal_t interface defined in sh2_hal.h.
 *
 * Hardware configuration:
 *   I2C peripheral : I2C1, 400 kHz
 *   SCL = PB6  SDA = PB7  (AF4, configured by HAL_I2C_MspInit)
 *   BNO085 address : 0x4A (7-bit) = PS0=PS1=GND (Adafruit default)
 *   RST = PD15  (active LOW output, idle HIGH — configured by MX_GPIO_Init)
 *   INT = PD14  (active LOW input, pull-up   — configured by MX_GPIO_Init)
 *
 * FreeRTOS notes:
 *   - All sh2 calls must come from a single task (sh2 is not re-entrant).
 *   - hal_open() uses osDelay() during the hardware reset sequence.
 *   - I2C1 is exclusively owned by the IMU task — no mutex is required.
 */

#include "sh2_hal_impl.h"
#include "sh2_hal.h"
#include "sh2_err.h"

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Hardware configuration
 * ---------------------------------------------------------------------- */
#define BNO085_I2C_HANDLE       hi2c1
#define BNO085_I2C_ADDR         ((uint16_t)(0x4A << 1))  /* 8-bit HAL address */
#define BNO085_I2C_TIMEOUT_MS   20

#define BNO085_RST_PIN          GPIO_PIN_15
#define BNO085_RST_PORT         GPIOD

#define BNO085_INT_PIN          GPIO_PIN_14
#define BNO085_INT_PORT         GPIOD

extern I2C_HandleTypeDef BNO085_I2C_HANDLE;

/* -------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------- */
static inline void rst_assert(void)
{
    HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_RESET);
}

static inline void rst_deassert(void)
{
    HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_SET);
}

static inline GPIO_PinState int_read(void)
{
    return HAL_GPIO_ReadPin(BNO085_INT_PORT, BNO085_INT_PIN);
}

/* Microsecond timestamp using DWT cycle counter. */
static uint32_t getTimeUs(void)
{
#if defined(DWT)
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
#else
    return HAL_GetTick() * 1000U;
#endif
}

/* -------------------------------------------------------------------------
 * sh2_Hal_t callbacks
 * ---------------------------------------------------------------------- */

/*
 * hal_open() — assert RST for 10 ms, release, then wait 300 ms for the
 * BNO085 to boot and send its initial SHTP advertisement.
 */
static int hal_open(sh2_Hal_t *self)
{
    (void)self;
    rst_assert();
    osDelay(10);
    rst_deassert();
    osDelay(300);
    return SH2_OK;
}

/* hal_close() — put sensor back in reset. */
static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    rst_assert();
}

/*
 * hal_read() — called by shtp_service() on every sh2_service() call.
 *
 * Returns 0 immediately if INT is not asserted (no data ready).
 * Two-phase I2C read: 4-byte SHTP header to learn packet length, then
 * the full packet. The BNO085 resets its internal read pointer at every
 * I2C START condition, so the second read re-presents from byte 0.
 */
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    (void)self;

    if (int_read() != GPIO_PIN_RESET) {
        return 0;
    }

    *t_us = getTimeUs();

    /* Phase 1: read 4-byte SHTP header to extract packet length */
    uint8_t hdr[4] = {0};
    if (HAL_I2C_Master_Receive(&BNO085_I2C_HANDLE, BNO085_I2C_ADDR,
                                hdr, 4, BNO085_I2C_TIMEOUT_MS) != HAL_OK) {
        return SH2_ERR_IO;
    }

    uint16_t rawLen    = (uint16_t)((hdr[1] << 8) | hdr[0]);
    uint16_t packetLen = rawLen & 0x7FFFu;   /* strip continuation flag */

    if (packetLen < 4 || packetLen > (uint16_t)len) {
        return 0;
    }

    /* Phase 2: read full packet; BNO085 rewinds to packet start on new START */
    if (HAL_I2C_Master_Receive(&BNO085_I2C_HANDLE, BNO085_I2C_ADDR,
                                pBuffer, packetLen, BNO085_I2C_TIMEOUT_MS) != HAL_OK) {
        return SH2_ERR_IO;
    }

    return (int)packetLen;
}

/*
 * hal_write() — send a SHTP packet over I2C.
 * No WAKE pin or INT polling needed in I2C mode.
 */
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    (void)self;

    if (HAL_I2C_Master_Transmit(&BNO085_I2C_HANDLE, BNO085_I2C_ADDR,
                                  pBuffer, (uint16_t)len,
                                  BNO085_I2C_TIMEOUT_MS) != HAL_OK) {
        return SH2_ERR_IO;
    }

    return (int)len;
}

/* hal_getTimeUs() — free-running 32-bit microsecond counter. */
static uint32_t hal_getTimeUs(sh2_Hal_t *self)
{
    (void)self;
    return getTimeUs();
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */
static sh2_Hal_t bno085_hal = {
    .open      = hal_open,
    .close     = hal_close,
    .read      = hal_read,
    .write     = hal_write,
    .getTimeUs = hal_getTimeUs,
};

sh2_Hal_t *BNO085_GetHal(void)
{
    return &bno085_hal;
}
