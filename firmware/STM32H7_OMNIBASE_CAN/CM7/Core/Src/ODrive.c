/*
 * ODrive.c
 *
 *  Created on: 06-Mar-2026
 *      Author: roger
 *  BASED ON FOLLOWING REPO LINK: https://github.com/siddarthiyer/ODrive-STM32-CAN-Driver/blob/main/ODrive/ODrive.c
 */


#include <ODrive.h>

extern FDCAN_HandleTypeDef hfdcan1;


static inline void pack_i16(uint8_t *dst, int16_t value) {
    memcpy(dst, &value, sizeof(int16_t));
}

static inline void pack_f32(uint8_t *dst, float value) {
    memcpy(dst, &value, sizeof(float));
}

static inline void pack_u32(uint8_t *dst, uint32_t value) {
    memcpy(dst, &value, sizeof(uint32_t));
}

static inline uint32_t unpack_u32(const uint8_t *src) {
    uint32_t value;
    memcpy(&value, src, sizeof(uint32_t));
    return value;
}

static inline int32_t unpack_i32(const uint8_t *src) {
    int32_t value;
    memcpy(&value, src, sizeof(int32_t));
    return value;
}

static inline float unpack_f32(const uint8_t *src) {
    float value;
    memcpy(&value, src, sizeof(float));
    return value;
}

static inline uint8_t fdcan_dlc_to_bytes(uint32_t dlc) {
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0;
    case FDCAN_DLC_BYTES_1:  return 1;
    case FDCAN_DLC_BYTES_2:  return 2;
    case FDCAN_DLC_BYTES_3:  return 3;
    case FDCAN_DLC_BYTES_4:  return 4;
    case FDCAN_DLC_BYTES_5:  return 5;
    case FDCAN_DLC_BYTES_6:  return 6;
    case FDCAN_DLC_BYTES_7:  return 7;
    case FDCAN_DLC_BYTES_8:  return 8;
    // (FD sizes omitted since you’re using classic CAN)
    default: return 0;
  }
}


void Set_TX_Param(FDCAN_TxHeaderTypeDef *tx, uint8_t node_id, uint8_t command_id, uint32_t id_type, uint32_t frame_type, uint32_t data_length) {
    tx->Identifier  = ((node_id & 0x3F) << 5) | (command_id & 0x1F);
    tx->IdType      = id_type;
    tx->TxFrameType = frame_type;
    tx->DataLength  = data_length;
}

HAL_StatusTypeDef Set_Axis_Requested_State(const Axis *axis, FDCAN_TXmsg *msg, uint8_t state) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_AXIS_REQUESTED_STATE, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	uint32_t Requested_State = state;
	pack_u32(&msg->data[0], Requested_State);
	msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Set_Input_Vel(const Axis *axis, FDCAN_TXmsg *msg, float vel, float torqueff) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_INPUT_VEL, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	pack_f32(&msg->data[0], vel);
	pack_f32(&msg->data[4], torqueff);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Clear_Errors(const Axis *axis, FDCAN_TXmsg *msg) {
	Set_TX_Param(&msg->header, axis->NODE_ID, CLEAR_ERRORS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_0);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Reboot_ODrive(const Axis *axis, FDCAN_TXmsg *msg) {
	Set_TX_Param(&msg->header, axis->NODE_ID, REBOOT_ODRIVE, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_0);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Set_Controller_Modes(const Axis *axis, FDCAN_TXmsg *msg, Control_Mode ControlMode, Input_Mode InputMode) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_CONTROLLER_MODES, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	pack_u32(&msg->data[0], (uint32_t)ControlMode);
	pack_u32(&msg->data[4], (uint32_t)InputMode);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Set_Input_Pos(const Axis *axis, FDCAN_TXmsg *msg, float Input_Pos, int16_t Vel_FF, int16_t Torque_FF) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_INPUT_POS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	pack_f32(&msg->data[0], Input_Pos);
	pack_i16(&msg->data[4], Vel_FF);
	pack_i16(&msg->data[6], Torque_FF);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Get_Encoder_Count(const Axis *axis, FDCAN_TXmsg *msg) {
	Set_TX_Param(&msg->header, axis->NODE_ID, GET_ENCODER_COUNT, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Set_Input_Torque(const Axis *axis, FDCAN_TXmsg *msg, float torque) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_INPUT_TORQUE, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	pack_f32(&msg->data[0], torque);
	msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}


HAL_StatusTypeDef Get_Bus_Voltage_Current(const Axis *axis, FDCAN_TXmsg *msg) {
	Set_TX_Param(&msg->header, axis->NODE_ID, GET_BUS_VOLTAGE_CURRENT, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Get_IQ(const Axis *axis, FDCAN_TXmsg *msg) {
	Set_TX_Param(&msg->header, axis->NODE_ID, GET_IQ, FDCAN_STANDARD_ID, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}


HAL_StatusTypeDef Set_Position_Gain(const Axis *axis, FDCAN_TXmsg *msg, float pos_gain) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_POSITION_GAIN, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	pack_f32(&msg->data[0], pos_gain);
	msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}


HAL_StatusTypeDef Set_Vel_Gains(const Axis *axis, FDCAN_TXmsg *msg, float Vel_Gain, float Vel_Int_Gain) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_VEL_GAINS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	pack_f32(&msg->data[0], Vel_Gain);
	pack_f32(&msg->data[4], Vel_Int_Gain);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Set_Axis_Node_ID(const Axis *axis, FDCAN_TXmsg *msg, uint32_t node_id) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_AXIS_NODE_ID, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_4);
	pack_u32(&msg->data[0], node_id);
	msg->data[4] = 0; msg->data[5] = 0; msg->data[6] = 0; msg->data[7] = 0;
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

HAL_StatusTypeDef Set_Limits(const Axis *axis, FDCAN_TXmsg *msg, float vel_lim, float curr_lim) {
	Set_TX_Param(&msg->header, axis->NODE_ID, SET_LIMITS, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8);
	pack_f32(&msg->data[0], vel_lim);
	pack_f32(&msg->data[4], curr_lim);
	return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg->header, msg->data);
}

void ODrive_RX_CallBack(Axis *axis, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t RX[8]) {
	uint32_t id = RxHeader->Identifier;
	uint8_t node_id = (uint8_t)(id >> 5);
	uint8_t cmd_id  = (uint8_t)(id & 0x1F);

	if (node_id != axis->NODE_ID) {
		return;
	}

	switch (cmd_id) {

		case ODRIVE_HEARTBEAT_MESSAGE:
			axis->AXIS_Error         = unpack_u32(&RX[0]);
			axis->AXIS_Current_State = RX[4];
			axis->Controller_Status  = RX[5];
//			axis->UPDATED = 1;
			break;

		case ENCODER_ESTIMATES:
			axis->AXIS_Encoder_Pos = unpack_f32(&RX[0]);
			axis->AXIS_Encoder_Vel = unpack_f32(&RX[4]);
			axis->UPDATED = 1;
			break;

		case GET_ENCODER_COUNT:
			axis->AXIS_Encoder_Shadow = unpack_i32(&RX[0]);
			axis->AXIS_Encoder_CPR    = unpack_i32(&RX[4]);
			axis->UPDATED = 1;
			break;

		case GET_BUS_VOLTAGE_CURRENT:
			axis->AXIS_Bus_Voltage = unpack_f32(&RX[0]);
			axis->AXIS_Bus_Current = unpack_f32(&RX[4]);
			axis->UPDATED = 1;
			break;

		case GET_IQ:
			axis->AXIS_Iq_Setpoint = unpack_f32(&RX[0]);
			axis->AXIS_Iq_Measured = unpack_f32(&RX[4]);
			axis->UPDATED = 1;
			break;
	}
}
