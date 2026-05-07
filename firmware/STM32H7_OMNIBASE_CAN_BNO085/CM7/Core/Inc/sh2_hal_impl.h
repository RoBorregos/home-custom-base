/*
 * sh2_hal_impl.h
 *
 * Public header for the STM32H7 SH2 HAL implementation.
 * Include this in main.c (or wherever you call sh2_open).
 */

#ifndef SH2_HAL_IMPL_H
#define SH2_HAL_IMPL_H

#include "sh2_hal.h"

/**
 * @brief Returns a pointer to the sh2_Hal_t struct for the BNO085.
 *        Pass this directly to sh2_open().
 *
 * Example:
 *   sh2_open(BNO085_GetHal(), eventCallback, NULL);
 */
sh2_Hal_t *BNO085_GetHal(void);

#endif /* SH2_HAL_IMPL_H */
