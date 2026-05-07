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
#define BTN_SPEED_DOWN     0x01
#define BTN_TOGGLE_UART    0x02
#define BTN_ESTOP          0x04
#define BTN_SPEED_UP       0x08

// Local-only buttons — stripped before forwarding to STM32.
// BTN_TOGGLE_UART is also local: the state is communicated via the
// controll_state field, not by forwarding the button bit.
#define BTN_LOCAL_MASK     (BTN_SPEED_DOWN | BTN_TOGGLE_UART | BTN_SPEED_UP)

// ── Velocity settings ─────────────────────────────────────────────────────────
#define MAX_SPEED_DEFAULT  0.12f
#define MAX_SPEED_STEP     0.03f
#define MAX_SPEED_MIN      0.03f

// ── STM32 UART ────────────────────────────────────────────────────────────────
// ESP32 GPIO17 TX → STM32 PD6 USART2 RX
// ESP32 GPIO16 RX → STM32 PD5 USART2 TX
#define STM32_UART_PORT  UART_NUM_2
#define STM32_UART_TX    GPIO_NUM_17
#define STM32_UART_RX    GPIO_NUM_16
#define STM32_BAUD       115200
#define STM32_UART_BUF   1024

// Heartbeat period — STM32 always knows the connection state within this window.
#define HEARTBEAT_MS     100

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

static bool     uart_tx_enabled          = false;
static float    current_max_speed        = MAX_SPEED_DEFAULT;
static bool     estop_active             = false;
static bool     any_controller_connected = false;
static uint32_t last_packet_ms           = 0;

/*
 * controll_state encodes the full ESP32 status in one field:
 *   0 = no controller paired
 *   1 = controller paired, TX disabled  (STM32 BT_active = 1 "Paired")
 *   2 = controller paired, TX enabled   (STM32 BT_active = 2 "Active")
 *
 * STM32 maps this directly to BT_active — no button-based signalling needed.
 */
static uint8_t controllState() {
    if (!any_controller_connected) return 0;
    return uart_tx_enabled ? 2 : 1;
}

// Protocol: "3 <vx> <vy> <wz> <buttons_hex> <controll_state>\r\n"
static void sendToSTM32(float vx, float vy, float wz, uint8_t buttons,
                        uint8_t controll_state) {
    char buf[72];
    int len = snprintf(buf, sizeof(buf),
                       "3 %.3f %.3f %.3f %02X %u\r\n",
                       vx, vy, wz, buttons, controll_state);
    uart_write_bytes(STM32_UART_PORT, buf, len);
    last_packet_ms = millis();
}

// Map Bluepad32 raw axis [-511, 512] to velocity [-maxVel, maxVel]
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
            any_controller_connected = true;
            // Notify STM32 immediately (state = 1 "Paired", TX still off)
            sendToSTM32(0.0f, 0.0f, 0.0f, 0x00, controllState());
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
        }
    }
    // Recheck whether any controller remains
    any_controller_connected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr) {
            any_controller_connected = true;
            break;
        }
    }
    // Reset TX state so reconnection always starts as Paired, not Active
    if (!any_controller_connected) {
        uart_tx_enabled = false;
        estop_active    = false;
    }
    // Notify STM32 immediately (state = 0 "Inactive" if none remain)
    sendToSTM32(0.0f, 0.0f, 0.0f, 0x00, controllState());
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
        uart_tx_enabled = !uart_tx_enabled;
        estop_active    = false;
        Serial.printf("UART TX %s\n", uart_tx_enabled ? "ENABLED" : "DISABLED");

        /*
         * Send an immediate state packet so STM32 transitions BT_active
         * without waiting up to HEARTBEAT_MS.
         * controll_state=2 → STM32 BT_active=2 (Active)
         * controll_state=1 → STM32 BT_active=1 (Paired), queues stop command
         */
        prev_buttons = buttons;
        sendToSTM32(0.0f, 0.0f, 0.0f, 0x00, controllState());
        return;
    }

    // ── Speed scaling buttons ────────────────────────────────────────────────
    if (speed_up_pressed) {
        current_max_speed += MAX_SPEED_STEP;
        Serial.printf("Max speed increased: %.2f\n", current_max_speed);
    }

    if (speed_down_pressed) {
        current_max_speed -= MAX_SPEED_STEP;
        if (current_max_speed < MAX_SPEED_MIN)
            current_max_speed = MAX_SPEED_MIN;
        Serial.printf("Max speed decreased: %.2f\n", current_max_speed);
    }

    prev_buttons = buttons;

    // ── E-stop ───────────────────────────────────────────────────────────────
    estop_active = (buttons & BTN_ESTOP) != 0;

    /*
     * If TX is disabled: do not send motion packets.
     * The heartbeat in loop() keeps sending controll_state=1 so STM32
     * continues to show "Paired" without queuing motion commands.
     */
    if (!uart_tx_enabled) {
        return;
    }

    // controll_state=2 in all motion packets (we only reach here when enabled)
    uint8_t fwd_buttons = buttons & ~(uint8_t)BTN_LOCAL_MASK;

    if (estop_active) {
        sendToSTM32(0.0f, 0.0f, 0.0f, fwd_buttons, 2);
        Serial.printf("ESTOP: buttons=0x%02X max=%.2f\n",
                      fwd_buttons, current_max_speed);
        return;
    }

    float vx = axisToVel(-ctl->axisY(),  current_max_speed);
    float vy = axisToVel(-ctl->axisX(),  current_max_speed);
    float wz = axisToVel(-ctl->axisRX(), current_max_speed);

    sendToSTM32(vx, vy, wz, fwd_buttons, 2);

    Serial.printf("vx=%.3f vy=%.3f wz=%.3f buttons=0x%02X raw=0x%02X max=%.2f\n",
                  vx, vy, wz, fwd_buttons, buttons, current_max_speed);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController &&
            myController->isConnected() &&
            myController->hasData()) {
            if (myController->isGamepad())
                processGamepad(myController);
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
                                 STM32_UART_TX, STM32_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(STM32_UART_PORT,
                                        STM32_UART_BUF, STM32_UART_BUF,
                                        0, NULL, 0));

#if BT_STUB
    Serial.println("BT_STUB=1: sending fixed values, no controller needed");
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
                  BTN_SPEED_DOWN, BTN_TOGGLE_UART, BTN_ESTOP, BTN_SPEED_UP);
    Serial.println("Waiting for Bluetooth controller...");
#endif
}

void loop() {
#if BT_STUB
    // Stub always sends as Active (state=2)
    sendToSTM32(STUB_VX, STUB_VY, STUB_WZ, STUB_BUTTONS, 2);
    Serial.printf("[STUB] vx=%.3f vy=%.3f wz=%.3f buttons=0x%02X\n",
                  STUB_VX, STUB_VY, STUB_WZ, STUB_BUTTONS);
    delay(STUB_PERIOD_MS);
#else
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // Safety resend while e-stop is held
    if (estop_active && uart_tx_enabled)
        sendToSTM32(0.0f, 0.0f, 0.0f, BTN_ESTOP, 2);

    /*
     * Heartbeat: guarantee STM32 receives the current state at least every
     * HEARTBEAT_MS even when there is no controller input (e.g. idle sticks
     * or TX disabled).  sendToSTM32 updates last_packet_ms, so if
     * processControllers already sent a packet this cycle the heartbeat
     * is skipped.
     */
    uint32_t now = millis();
    if (now - last_packet_ms >= HEARTBEAT_MS)
        sendToSTM32(0.0f, 0.0f, 0.0f, 0x00, controllState());

    vTaskDelay(10);
#endif
}
