/*
 * ODrive.h
 *
 *  Created on: 06-Mar-2026
 *      Author: roger
 *  BASED ON REPO: https://github.com/siddarthiyer/ODrive-STM32-CAN-Driver/blob/main/ODrive/ODrive.h
 */

#ifndef ODRIVE_ODRIVE_H_
#define ODRIVE_ODRIVE_H_

#include "main.h"
#include <stdint.h>
#include <string.h>

/* COMMAND ID */
#define ODRIVE_HEARTBEAT_MESSAGE        0x001
#define SET_AXIS_NODE_ID                0x006
#define SET_AXIS_REQUESTED_STATE        0x007
#define ENCODER_ESTIMATES               0x009
#define GET_ENCODER_COUNT               0x00A
#define SET_CONTROLLER_MODES            0x00B
#define SET_INPUT_POS                   0x00C
#define SET_INPUT_VEL                   0x00D
#define SET_INPUT_TORQUE                0x00E
#define SET_LIMITS                      0x00F
#define GET_IQ                          0x014
#define REBOOT_ODRIVE                   0x016
#define GET_BUS_VOLTAGE_CURRENT         0x017
#define CLEAR_ERRORS                    0x018
#define SET_POSITION_GAIN               0x01A
#define SET_VEL_GAINS                   0x01B

/* Arbitrary parameter access (CANSimple "RxSdo"/"TxSdo", fw v0.6.x+).
 * Lets the host write any endpoint in the ODrive's flat parameter tree
 * (e.g. axis0.controller.config.vel_ramp_rate) without a dedicated fixed
 * command. Endpoint IDs are firmware-build-specific -- look them up in the
 * flat_endpoints.json shipped with the ODrive's exact firmware release. */
#define RXSDO_CMD_ID                    0x004
#define SDO_OPCODE_READ                 0x00
#define SDO_OPCODE_WRITE                0x01

extern volatile uint8_t BT_active;

/* Axis Parameters */
typedef struct Axis
{
    uint8_t  NODE_ID;
    float    AXIS_Encoder_Pos;
    float    AXIS_Encoder_Vel;
    int32_t  AXIS_Encoder_CPR;
    int32_t  AXIS_Encoder_Shadow;
    float    AXIS_Bus_Voltage;
    float    AXIS_Bus_Current;
    float    AXIS_Iq_Setpoint;
    float    AXIS_Iq_Measured;
    uint32_t AXIS_Error;
    uint8_t  AXIS_Current_State;
    uint8_t  Controller_Status;
    volatile uint8_t UPDATED;
    uint8_t  gear_ratio;
    /* The axis state that counts as "armed and healthy" for this wheel.
     * Every wheel except node 33 uses true closed-loop velocity control
     * (native encoder + onboard PI), so Armed_State == CLOSED_LOOP_CONTROL.
     * Node 33 (the v3.6 replacement for the dead S1) has no commutation-grade
     * encoder -- only an external, low-resolution Hall sensor unsuited for
     * FOC feedback -- so it runs forced-commutation open loop (custom firmware,
     * see odrive_config/firmware/) and its healthy/armed state is LOCKIN_SPIN
     * instead. Set once per axis where NODE_ID/gear_ratio are assigned. */
    uint8_t  Armed_State;
} Axis;

/* Axis States */
typedef enum {
    UNDEFINED = 0x0,
    IDLE = 0x1,
    STARTUP_SEQUENCE = 0x2,
    FULL_CALIBRATION_SEQUENCE = 0x3,
    MOTOR_CALIBRATION = 0x4,
    ENCODER_INDEX_SEARCH = 0x6,
    ENCODER_OFFSET_CALIBRATION = 0x7,
    CLOSED_LOOP_CONTROL = 0x8,
    LOCKIN_SPIN = 0x9,
    ENCODER_DIR_FIND = 0xA,
    HOMING = 0xB,
    ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
    ENCODER_HALL_PHASE_CALIBRATION = 0xD
} Axis_State;

/* Control Modes */
typedef enum {
    VOLTAGE_CONTROL = 0x0,
    TORQUE_CONTROL = 0x1,
    VELOCITY_CONTROL = 0x2,
    POSITION_CONTROL = 0x3
} Control_Mode;

/* Input Modes */
typedef enum {
    INACTIVE = 0x0,
    PASSTHROUGH = 0x1,
    VEL_RAMP = 0x2,
    POS_FILTER = 0x3,
    MIX_CHANNELS = 0x4,
    TRAP_TRAJ = 0x5,
    TORQUE_RAMP = 0x6,
    MIRROR = 0x7,
    TUNING = 0x8
} Input_Mode;

typedef struct
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} FDCAN_TXmsg;

/* Core helper */
void Set_TX_Param(FDCAN_TxHeaderTypeDef *tx, uint8_t node_id, uint8_t command_id, uint32_t id_type, uint32_t frame_type, uint32_t data_length);

/* TX command functions */
HAL_StatusTypeDef Set_Axis_Requested_State(const Axis *axis, FDCAN_TXmsg *msg, uint8_t state);

HAL_StatusTypeDef Set_Input_Vel(const Axis *axis, FDCAN_TXmsg *msg, float vel, float torqueff);

HAL_StatusTypeDef Clear_Errors(const Axis *axis, FDCAN_TXmsg *msg);

HAL_StatusTypeDef Reboot_ODrive(const Axis *axis, FDCAN_TXmsg *msg);

HAL_StatusTypeDef Set_Controller_Modes(const Axis *axis, FDCAN_TXmsg *msg, Control_Mode ControlMode, Input_Mode InputMode);

HAL_StatusTypeDef Set_Input_Pos(const Axis *axis, FDCAN_TXmsg *msg, float Input_Pos, int16_t Vel_FF, int16_t Torque_FF);

HAL_StatusTypeDef Get_Encoder_Count(const Axis *axis, FDCAN_TXmsg *msg);

HAL_StatusTypeDef Set_Input_Torque(const Axis *axis, FDCAN_TXmsg *msg, float torque);

HAL_StatusTypeDef Get_Bus_Voltage_Current(const Axis *axis, FDCAN_TXmsg *msg);

HAL_StatusTypeDef Get_IQ(const Axis *axis, FDCAN_TXmsg *msg);

HAL_StatusTypeDef Set_Position_Gain(const Axis *axis, FDCAN_TXmsg *msg, float pos_gain);

HAL_StatusTypeDef Set_Vel_Gains(const Axis *axis, FDCAN_TXmsg *msg, float Vel_Gain, float Vel_Int_Gain);

/* Arbitrary parameter write (RxSdo, OPCODE_WRITE) -- sets a single float32
 * endpoint by its flat_endpoints.json ID. See RXSDO_CMD_ID comment above. */
HAL_StatusTypeDef Set_Param_Float(const Axis *axis, FDCAN_TXmsg *msg, uint16_t endpoint_id, float value);

HAL_StatusTypeDef Set_Axis_Node_ID(const Axis *axis, FDCAN_TXmsg *msg, uint32_t node_id);

HAL_StatusTypeDef Set_Limits(const Axis *axis, FDCAN_TXmsg *msg, float vel_lim, float curr_lim);

/* RX decode callback */
void ODrive_RX_CallBack(Axis *axis, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t RX[8]);

#endif /* ODRIVE_ODRIVE_H_ */
