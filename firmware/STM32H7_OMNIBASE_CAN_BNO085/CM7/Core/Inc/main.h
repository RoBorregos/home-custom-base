/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* Default ODrive controller gains pushed by ODrive_Startup, one per axis.
 *   pos_gain     [(rev/s) / rev]   — position-loop P
 *   vel_gain     [Nm / (rev/s)]    — velocity-loop P
 *   vel_int_gain [Nm / rev]        — velocity-loop I
 * Mirror the dashboard slider defaults in
 * omnibase_ws/src/odrive_comm/assets/dashboard.html (pgv/vgv/vigv). */
#define ODRIVE_DEFAULT_POS_GAIN      { 20.00f, 20.00f, 20.00f, 20.00f }
#define ODRIVE_DEFAULT_VEL_GAIN      {  0.32f,  0.32f,  0.32f,  0.32f }
#define ODRIVE_DEFAULT_VEL_INT_GAIN  {  0.70f,  0.70f,  0.70f,  0.70f }

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

#define PD7_Pin GPIO_PIN_7
#define PD6_Pin GPIO_PIN_6
#define PD5_Pin GPIO_PIN_5
#define PD4_Pin GPIO_PIN_4

#define PE2_Pin GPIO_PIN_2
#define PE3_Pin GPIO_PIN_3
#define PE4_Pin GPIO_PIN_4
#define PE6_Pin GPIO_PIN_6

#define BT_UART_TOGGLE_BUTTON  0x02
#define BT_ESTOP_BUTTON 0x04

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDGains;

typedef struct {
    PIDGains x_pid;
    PIDGains y_pid;
    PIDGains phi_pid;
    PIDGains u_pid[4];  // Per-wheel PID
} PIDConfig;

typedef struct
{
	double x_desired;
	double y_desired;
	double phi_end;
	double d;
	double x_off;
	double y_off;
	double r;
//	uint8_t state;
	float u1_desired;
	float u2_desired;
	float u3_desired;
	float u4_desired;

} InputData;

typedef struct {
    /* Euler angles (degrees) — kept for backward compatibility with the dashboard. */
    double yaw;
    double roll;
    double pitch;

    /* Orientation quaternion from BNO085 SH2_ROTATION_VECTOR.
     * Standard convention: (qx, qy, qz, qw) with qw being the real component. */
    float qx;
    float qy;
    float qz;
    float qw;

    /* Angular velocity from SH2_GYROSCOPE_CALIBRATED, units rad/s, body frame. */
    float wx;
    float wy;
    float wz;

    /* Linear acceleration from SH2_LINEAR_ACCELERATION (gravity removed),
     * units m/s^2, body frame. */
    float ax;
    float ay;
    float az;
} IMUData;

typedef struct {
    double err_x;
    double err_y;
    double err_phi;
    double u_errs[4];
} Errors;

typedef struct {
    uint32_t current;
    uint32_t previous;
    uint32_t delta;
    uint32_t print_prev;
} TimeState;

typedef struct {
	uint16_t cnt_vals[4];
	uint16_t prevcnt_vals[4];
	float angleVals[4];
	double omegaVals[4];
} EncoderData;

typedef struct {
	/* Legacy fields, kept so the existing UART/dashboard pipeline keeps
	 * working. After the EKF integration these are populated from the
	 * filtered state, not from raw dead-reckoning. q_dot stores the
	 * world-frame velocity tuple [phi_dot, x_dot_world, y_dot_world]. */
	double x_pos;
	double y_pos;
	double phi;
	double q_dot[3];

	/* Planar mobile base: z is always 0. */
	double z_pos;

	/* Orientation quaternion derived from EKF yaw (planar => qx=qy=0). */
	float qx;
	float qy;
	float qz;
	float qw;

	/* Body-frame twist (vz, wx, wy are always 0 for a mecanum base). */
	double vx_body;
	double vy_body;
	double vz_body;
	double wx;
	double wy;
	double wz;

	/* ROS-style row-major 6x6 covariance matrices.
	 *   pose_covariance axes  : (x, y, z, roll, pitch, yaw)
	 *   twist_covariance axes : (vx, vy, vz, wx, wy, wz)
	 * Unobserved planar-base axes carry a large variance sentinel (1e6). */
	double pose_covariance[36];
	double twist_covariance[36];
} OdomData;

typedef struct {
  double x_dot;   // desired x velocity
  double y_dot;   // desired y velocity
  double phi_dot;   // desired rotational velocity
  double PWM_vals[4]; // Motor PIDs outputs
  uint16_t dutyCycles[4]; // Motor control limited duty cycles
  uint8_t M_dirs[4]; // Motor directions,  0 = forward, 1 = back <- arbitrary
  double u[4]; // ik computed required wheel speeds (wheel velocity control input)
} CtrlOutData;


typedef struct {
    IMUData imu;
    EncoderData encoders;
    Errors error;
    TimeState time;
    OdomData odom;
    CtrlOutData ctrl;
} CtrlTsk_Data;

typedef enum {
    /* Runtime control commands */
    ODRIVE_CMD_NONE                = 0x00,
    ODRIVE_CMD_SET_VEL             = 0x01,
    ODRIVE_CMD_SET_STATE           = 0x02,
    ODRIVE_CMD_CLEAR_ERRORS        = 0x03,
    ODRIVE_CMD_SET_CONTROLLER_MODE = 0x04,
    ODRIVE_CMD_SET_LIMITS          = 0x05,
    ODRIVE_CMD_SET_POS             = 0x06,
    ODRIVE_CMD_STOP_ODRIVES        = 0x07,
    /* Configuration / state-machine sub-commands */
    ODRIVE_CFG_CLEAR_ERRORS        = 0x20,
    ODRIVE_CFG_SET_STATE           = 0x21,
    ODRIVE_CFG_SET_CTRL_MODE       = 0x22,
    ODRIVE_CFG_SET_LIMITS          = 0x23,
    ODRIVE_CFG_SET_POS_GAIN        = 0x24,
    ODRIVE_CFG_SET_VEL_GAINS       = 0x25,
    ODRIVE_CFG_STARTUP             = 0x26,
    ODRIVE_CFG_REBOOT              = 0x27,
    ODRIVE_CFG_SET_TORQUE          = 0x28,
    ODRIVE_CFG_STOP                = 0x29,
    ODRIVE_CFG_SET_INPUT_POS       = 0x30,
} ODriveCmdType;

typedef enum {
    SM_BOOT    = 0,
    SM_STARTUP = 1,
    SM_RUNNING = 2,
    SM_IDLE    = 3,
} ODriveSMState;

typedef enum {
    CMD_SOURCE_ROS = 1,
    CMD_SOURCE_BT  = 2,
} ODriveCmdSource;

typedef struct {
    ODriveCmdType type;
    uint8_t target_mask;     // bitmask or 0xFF for all axes
    double robot_twist[3];    // used for set vel
    double torque_ff[4];      // optional
    uint8_t axis_state;      // used for set state
    uint32_t control_mode;   // if needed
    uint32_t input_mode;     // if needed
    float vel_limit;
    float curr_limit;
    float pos_gain;
    float vel_gain;
    float vel_int_gain;
    float input_pos_target;
    float input_pos_vel_ff;
    float input_pos_trq_ff;
    uint8_t  source;
    uint16_t buttons;
} ODriveCmdMsg;

typedef struct
{
    uint8_t  node_id[4];

    uint32_t axis_error[4];
    uint8_t  axis_state[4];
    uint8_t  controller_status[4];

    float pos_est[4];
    float vel_est[4];

    int32_t encoder_shadow[4];
    int32_t encoder_cpr[4];

    float iq_setpoint[4];
    float iq_measured[4];

    float bus_voltage[4];
    float bus_current[4];

    uint8_t updated[4];

    double IK_computed_wheel_speeds[4];

    IMUData imu;

    OdomData odom;

    uint32_t timestamp_ms;
    float    bt_vx;
    float    bt_vy;
    float    bt_wz;
    uint8_t  bt_active;
} ODriveTelemetryMsg;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define MCP2515_CS_Pin GPIO_PIN_14
#define MCP2515_CS_GPIO_Port GPIOD

#define PI 3.141592

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
