/**
 ******************************************************************************
 * @file    bt_arbitration.h
 * @brief   BT (ESP32 pad) vs ROS source priority — pure predicates.
 *
 * Split out of main.c's SM_RUNNING loop so the arbitration can be unit tested
 * on the host without FreeRTOS or hardware: see Test/test_bt_arbitration.c.
 *
 * The ESP32 sends a Type-3 packet every HEARTBEAT_MS (100 ms) even with the
 * sticks centred, so "any BT packet claims the override" holds control forever
 * and drops every ROS command while the pad is on. Only a MOVING BT packet may
 * claim it.
 ******************************************************************************
 */
#ifndef BT_ARBITRATION_H
#define BT_ARBITRATION_H

#include <stdint.h>

/* BT_active values as reported by the ESP32 (controll_state):
 *   0 = no controller paired, 1 = paired / TX disabled, 2 = paired / TX enabled */
#define BT_STATE_INACTIVE 0u
#define BT_STATE_PAIRED   1u
#define BT_STATE_ACTIVE   2u

/**
 * @brief True when a BT packet should claim (or refresh) the motion override.
 *
 * Requires the pad to be Active AND actually commanding motion. BT_RX_Task
 * applies BT_DEADZONE_* before queueing, so a centred stick arrives as an
 * exact 0.0 and no epsilon is needed here.
 */
static inline uint8_t BT_ClaimsOverride(uint8_t bt_active, const double twist[3])
{
    return (uint8_t)(bt_active == BT_STATE_ACTIVE &&
                     (twist[0] != 0.0 || twist[1] != 0.0 || twist[2] != 0.0));
}

/**
 * @brief True while a held BT override must keep masking ROS commands.
 *
 * Tick subtraction is unsigned so it is correct across the 32-bit wrap.
 */
static inline uint8_t BT_HoldsOverride(uint8_t override_active,
                                      uint32_t now, uint32_t last_bt_tick,
                                      uint32_t timeout_ms)
{
    return (uint8_t)(override_active && (now - last_bt_tick) < timeout_ms);
}

#endif /* BT_ARBITRATION_H */
