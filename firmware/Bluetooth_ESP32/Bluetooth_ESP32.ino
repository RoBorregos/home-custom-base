#include <Bluepad32.h>
#include "driver/uart.h"

// ── Stub mode ─────────────────────────────────────────────────────────────────
#define BT_STUB 0

#define STUB_VX        0.5f
#define STUB_VY        0.3f
#define STUB_WZ        0.2f
#define STUB_BUTTONS   0x00
#define STUB_PERIOD_MS 100

// ── Button mapping ────────────────────────────────────────────────────────────
// Change these here if your controller mapping changes.
#define BTN_SPEED_DOWN     0x01
#define BTN_TOGGLE_UART    0x02
#define BTN_ESTOP          0x04
#define BTN_SPEED_UP       0x08

// Buttons used only by ESP32 logic.
// These are removed from normal packets sent to STM32.
#define BTN_LOCAL_MASK     (BTN_SPEED_DOWN | BTN_TOGGLE_UART | BTN_SPEED_UP)

// ── Velocity settings ─────────────────────────────────────────────────────────
#define MAX_SPEED_DEFAULT  0.12f
#define MAX_SPEED_STEP     0.03f
#define MAX_SPEED_MIN      0.03f

// ── STM32 UART ────────────────────────────────────────────────────────────────
// Wiring:
// ESP32 GPIO17 TX → STM32 PD6 USART2 RX
// ESP32 GPIO16 RX → STM32 PD5 USART2 TX
#define STM32_UART_PORT  UART_NUM_2
#define STM32_UART_TX    GPIO_NUM_17
#define STM32_UART_RX    GPIO_NUM_16
#define STM32_BAUD       115200
#define STM32_UART_BUF   1024

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

static bool uart_tx_enabled = false;
static float current_max_speed = MAX_SPEED_DEFAULT;
static bool estop_active = false;

// Send one velocity packet to the STM32:
// "3 <vx> <vy> <wz> <buttons_hex>\r\n"
static void sendToSTM32(float vx, float vy, float wz, uint8_t buttons) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf),
                       "3 %.3f %.3f %.3f %02X\r\n",
                       vx, vy, wz, buttons);
    uart_write_bytes(STM32_UART_PORT, buf, len);
}

// Map Bluepad32 raw axis value [-511, 512] to velocity [-maxVel, maxVel]
static float axisToVel(int32_t raw, float maxVel) {
    float norm = (float)raw / 512.0f;

    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    return norm * maxVel;
}

// ── Bluepad32 callbacks ───────────────────────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected, index=%d model=%s\n",
                          i, ctl->getModelName().c_str());
            myControllers[i] = ctl;
            return;
        }
    }

    Serial.println("Controller connected but no empty slot found");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            return;
        }
    }
}

void processGamepad(ControllerPtr ctl) {
    uint8_t buttons = (uint8_t)(ctl->buttons() & 0xFF);
    static uint8_t prev_buttons = 0;

    bool toggle_pressed =
        (buttons & BTN_TOGGLE_UART) && !(prev_buttons & BTN_TOGGLE_UART);

    bool speed_up_pressed =
        (buttons & BTN_SPEED_UP) && !(prev_buttons & BTN_SPEED_UP);

    bool speed_down_pressed =
        (buttons & BTN_SPEED_DOWN) && !(prev_buttons & BTN_SPEED_DOWN);

    // ── Toggle UART TX ───────────────────────────────────────────────────────
    if (toggle_pressed) {
        if (uart_tx_enabled) {
            /*
             * Currently enabled → now disabling.
             *
             * Send 3 stop/release packets with BTN_TOGGLE_UART.
             * STM32 should interpret this as:
             *   BT_active = 0;
             */
            Serial.println("UART TX DISABLING: sending 3 stop/release packets");

            for (int i = 0; i < 3; i++) {
                sendToSTM32(0.0f, 0.0f, 0.0f, BTN_TOGGLE_UART);

                Serial.printf("[BT OFF] stop/release packet %d buttons=0x%02X\n",
                              i + 1, BTN_TOGGLE_UART);

                delay(10);
            }

            uart_tx_enabled = false;
            estop_active = false;

            Serial.println("UART TX DISABLED");

            /*
             * Important:
             * Return immediately.
             * Do not send a normal motion packet in the same cycle.
             */
            prev_buttons = buttons;
            return;
        } else {
            /*
             * Currently disabled → now enabling.
             *
             * Do NOT send BTN_TOGGLE_UART here because STM32 uses it as
             * the release/disable command.
             */
            uart_tx_enabled = true;
            Serial.println("UART TX ENABLED");
        }
    }

    // ── Speed scaling buttons ────────────────────────────────────────────────
    // These can be pressed while moving. They only change local max speed.
    if (speed_up_pressed) {
        current_max_speed += MAX_SPEED_STEP;
        Serial.printf("Max speed increased: %.2f\n", current_max_speed);
    }

    if (speed_down_pressed) {
        current_max_speed -= MAX_SPEED_STEP;

        if (current_max_speed < MAX_SPEED_MIN) {
            current_max_speed = MAX_SPEED_MIN;
        }

        Serial.printf("Max speed decreased: %.2f\n", current_max_speed);
    }

    prev_buttons = buttons;

    // ── E-stop ───────────────────────────────────────────────────────────────
    estop_active = (buttons & BTN_ESTOP) != 0;

    /*
     * If UART TX is disabled, do not keep sending zeroes.
     * The 3 stop/release packets already told STM32 to release BT control.
     */
    if (!uart_tx_enabled) {
        return;
    }

    /*
     * Remove local-only buttons from normal STM32 packets.
     *
     * This fixes your bug:
     * pressing speed-up/speed-down while driving will no longer send 0x08/0x01
     * to STM32 as if they were robot command buttons.
     *
     * BTN_ESTOP is NOT removed, so STM32 can still see emergency stop.
     */
    uint8_t fwd_buttons = buttons & ~(uint8_t)BTN_LOCAL_MASK;

    if (estop_active) {
        sendToSTM32(0.0f, 0.0f, 0.0f, fwd_buttons);

        Serial.printf("ESTOP: vx=0.000 vy=0.000 wz=0.000 buttons=0x%02X max=%.2f\n",
                      fwd_buttons, current_max_speed);
        return;
    }

    // Left stick Y  → vx
    // Left stick X  → vy
    // Right stick X → wz
    float vx = axisToVel(-ctl->axisY(),  current_max_speed);
    float vy = axisToVel(-ctl->axisX(),  current_max_speed);
    float wz = axisToVel(-ctl->axisRX(), current_max_speed);

    sendToSTM32(vx, vy, wz, fwd_buttons);

    Serial.printf("vx=%.3f vy=%.3f wz=%.3f buttons=0x%02X raw_buttons=0x%02X max=%.2f\n",
                  vx, vy, wz, fwd_buttons, buttons, current_max_speed);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController &&
            myController->isConnected() &&
            myController->hasData()) {

            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// ── Arduino setup / loop ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    uart_config_t uart_config = {
        .baud_rate           = STM32_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_param_config(STM32_UART_PORT, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(STM32_UART_PORT,
                                 STM32_UART_TX,
                                 STM32_UART_RX,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(STM32_UART_PORT,
                                        STM32_UART_BUF,
                                        STM32_UART_BUF,
                                        0,
                                        NULL,
                                        0));

#if BT_STUB
    Serial.println("BT_STUB=1: sending fixed values to STM32, no controller needed");

    Serial.printf("vx=%.3f vy=%.3f wz=%.3f buttons=0x%02X period=%dms\n",
                  STUB_VX, STUB_VY, STUB_WZ, STUB_BUTTONS, STUB_PERIOD_MS);

    Serial.printf("UART%d TX=GPIO%d RX=GPIO%d baud=%d\n",
                  STM32_UART_PORT, STM32_UART_TX, STM32_UART_RX, STM32_BAUD);
#else
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    Serial.printf("Default max speed: %.2f\n", current_max_speed);
    Serial.printf("Buttons: SPEED_DOWN=0x%02X TOGGLE=0x%02X ESTOP=0x%02X SPEED_UP=0x%02X\n",
                  BTN_SPEED_DOWN,
                  BTN_TOGGLE_UART,
                  BTN_ESTOP,
                  BTN_SPEED_UP);

    Serial.println("Waiting for Bluetooth controller...");
#endif
}

void loop() {
#if BT_STUB
    sendToSTM32(STUB_VX, STUB_VY, STUB_WZ, STUB_BUTTONS);

    Serial.printf("[STUB] sent: vx=%.3f vy=%.3f wz=%.3f buttons=0x%02X\n",
                  STUB_VX, STUB_VY, STUB_WZ, STUB_BUTTONS);

    delay(STUB_PERIOD_MS);
#else
    bool dataUpdated = BP32.update();

    if (dataUpdated) {
        processControllers();
    }

    /*
     * Optional safety resend for e-stop.
     * This sends repeated zero packets while BTN_ESTOP is held.
     *
     * Notice it sends BTN_ESTOP, not 0x00 and not BTN_TOGGLE_UART.
     */
    if (estop_active && uart_tx_enabled) {
        sendToSTM32(0.0f, 0.0f, 0.0f, BTN_ESTOP);
    }

    vTaskDelay(10);
#endif
}