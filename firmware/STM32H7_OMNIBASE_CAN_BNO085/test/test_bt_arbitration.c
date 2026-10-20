/* Host-side self-check for the BT-vs-ROS source priority.
 * Build & run (from this directory, OUTSIDE the CubeIDE source paths):
 *   gcc -Wall -I../CM7/Core/Inc -o /tmp/t test_bt_arbitration.c && /tmp/t
 *
 * Regression under test: the ESP32 heartbeat (every 100 ms, zero twist,
 * controll_state=2) used to claim the motion override, which held it forever
 * and dropped every ROS command while the pad was paired -- the base only
 * accepted nav goals after the pad was switched off.
 */
#include <assert.h>
#include <stdio.h>
#include "bt_arbitration.h"

static const double STILL[3]   = {0.0,  0.0, 0.0};
static const double MOVING[3]  = {0.35, 0.0, 0.0};
static const double STRAFE[3]  = {0.0, -0.2, 0.0};
static const double SPIN[3]    = {0.0,  0.0, 0.8};

#define TIMEOUT 500u

int main(void)
{
    /* --- claiming ---------------------------------------------------------- */
    /* The bug: an idle pad must NOT take control. */
    assert(!BT_ClaimsOverride(BT_STATE_ACTIVE,   STILL));
    /* Any axis alone is enough to take control. */
    assert( BT_ClaimsOverride(BT_STATE_ACTIVE,   MOVING));
    assert( BT_ClaimsOverride(BT_STATE_ACTIVE,   STRAFE));
    assert( BT_ClaimsOverride(BT_STATE_ACTIVE,   SPIN));
    /* Paired-but-TX-disabled and unpaired never take control, moving or not. */
    assert(!BT_ClaimsOverride(BT_STATE_PAIRED,   MOVING));
    assert(!BT_ClaimsOverride(BT_STATE_INACTIVE, MOVING));
    assert(!BT_ClaimsOverride(BT_STATE_PAIRED,   STILL));

    /* --- holding / releasing ---------------------------------------------- */
    /* Never held when it was never claimed -> ROS drives. */
    assert(!BT_HoldsOverride(0, 1000u, 1000u, TIMEOUT));
    /* Fresh claim masks ROS. */
    assert( BT_HoldsOverride(1, 1000u, 1000u, TIMEOUT));
    assert( BT_HoldsOverride(1, 1499u, 1000u, TIMEOUT));
    /* Expires exactly at the timeout -> ROS regains control. */
    assert(!BT_HoldsOverride(1, 1500u, 1000u, TIMEOUT));
    assert(!BT_HoldsOverride(1, 9000u, 1000u, TIMEOUT));
    /* Correct across the 32-bit tick wrap. */
    assert( BT_HoldsOverride(1, 100u, 0xFFFFFF00u, TIMEOUT)); /* 356 ms elapsed */
    assert(!BT_HoldsOverride(1, 700u, 0xFFFFFF00u, TIMEOUT)); /* 956 ms elapsed */

    /* --- the reported scenario, end to end -------------------------------- */
    /* Pad paired + TX enabled, sticks centred, heartbeat every 100 ms. */
    uint8_t held = 0; uint32_t last = 0;
    for (uint32_t t = 0; t < 5000u; t += 100u) {
        if (BT_ClaimsOverride(BT_STATE_ACTIVE, STILL)) { held = 1; last = t; }
        /* ROS must get through on every single tick. */
        assert(!BT_HoldsOverride(held, t, last, TIMEOUT));
    }
    /* Operator grabs the stick: BT takes over immediately. */
    if (BT_ClaimsOverride(BT_STATE_ACTIVE, MOVING)) { held = 1; last = 5000u; }
    assert(held && BT_HoldsOverride(held, 5000u, last, TIMEOUT));
    /* Stick released: heartbeats no longer refresh, so ROS is back after the
     * timeout (the zero packet itself still stops the base immediately). */
    assert( BT_HoldsOverride(held, 5400u, last, TIMEOUT));
    assert(!BT_HoldsOverride(held, 5500u, last, TIMEOUT));

    /* --- stopping packets must never be dropped ---------------------------
     * main.c drops a BT packet only when `!bt_override_active && !had_override`.
     * Model that predicate here so the stop paths stay covered. */
    #define DROPPED(had, active, twist) \
        (!(BT_ClaimsOverride((active), (twist)) || \
           ((active) == BT_STATE_ACTIVE && (had))) && !(had))

    /* Idle heartbeat, nothing held -> dropped, ROS keeps driving. */
    assert( DROPPED(0, BT_STATE_ACTIVE,   STILL));
    /* Stick released while the override was held -> executes, base stops NOW. */
    assert(!DROPPED(1, BT_STATE_ACTIVE,   STILL));
    /* TX toggled off mid-drive -> executes, base stops. */
    assert(!DROPPED(1, BT_STATE_PAIRED,   STILL));
    /* Pad disconnected mid-drive -> executes, base stops. */
    assert(!DROPPED(1, BT_STATE_INACTIVE, STILL));
    /* Moving packet -> always executes. */
    assert(!DROPPED(0, BT_STATE_ACTIVE,   MOVING));

    printf("bt_arbitration: all checks passed\n");
    return 0;
}
