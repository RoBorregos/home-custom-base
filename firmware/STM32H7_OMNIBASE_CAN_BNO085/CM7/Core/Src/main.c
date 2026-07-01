/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_current_bluetooth.c
  * @brief          : main.c + BT_RX_Task (ESP32/Bluetooth UART on USART2)
  *
  * HOW TO TEST WITHOUT ODRIVES:
  *   Set CAN_STUB to 1 (default below).  FDCAN switches to internal-loopback
  *   mode so CAN TX completes without needing an external node/ACK — the
  *   ODriveTask state machine runs, queues are exercised, and the BT UART
  *   path works normally.  Set CAN_STUB to 0 when ODrives are connected.
  *
  * BT PROTOCOL (USART2 @115200 = ESP32 link; USART3 @230400 = ROS dashboard):
  *   Type-3 message from ESP32: "3 <vx> <vy> <wz> <buttons_hex>\r\n"
  *   example: "3 0.5 0.0 0.0 00\r\n"
  *
  * SOURCE PRIORITY:
  *   BT (ESP32) overrides ROS for BT_OVERRIDE_TIMEOUT_MS (500 ms).
  *   If no BT packet arrives within that window the robot stops and ROS
  *   regains control.  Bit 3 of the buttons field is an emergency stop.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bno055_stm32.h"
#include "uart_rx.h"
#include <string.h>
//#include "mcp2515.h"
#include "ODrive.h"
#include "ekf.h"
/* BNO085 SH2 library */
#include "sh2_hal_impl.h"
#include "sh2.h"
#include "sh2_err.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "euler.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* Set to 1 to run without ODrives on the CAN bus.
   FDCAN switches to INTERNAL_LOOPBACK: TX completes without external ACK so
   nothing blocks.  The state machine and BT/UART paths work normally.
   Set to 0 for real hardware with ODrives connected. */
#define CAN_STUB 0

/* Set to 1 when the BNO055 IMU is physically connected on I2C1.
   Set to 0 to skip I2C1 init and all bno085 calls (euler stays {0,0,0}). */
#define IMU_ENABLED 1

/* Set to 1 to inject a synthetic constant body-frame velocity into the EKF
   instead of computing it from ODrive feedback.  Combined with CAN_STUB=1 and
   IMU_ENABLED=0 this lets the full predict+correct pipeline run on a bare
   STM32 with nothing connected.  EKF_STUB_VX/VY/WZ are in m/s and rad/s. */
#define EKF_STUB    0
#define EKF_STUB_VX  0.1   /* forward at 0.1 m/s */
#define EKF_STUB_VY  0.0
#define EKF_STUB_WZ  0.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Wait for a free FDCAN TX FIFO slot with a 50 ms hard timeout.
   Without this, a dead/missing ODrive freezes the task indefinitely. */
#define FDCAN_WAIT_TX_FREE() do { \
    uint32_t _t0 = osKernelGetTickCount(); \
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) { \
        if ((osKernelGetTickCount() - _t0) > 50u) { \
            FirmwareError_Push(FERR_CAN_TX_TIMEOUT, FERR_NO_AXIS, 0); break; \
        } \
        osDelay(1); \
    } \
} while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_RX_Task */
osThreadId_t UART_RX_TaskHandle;
const osThreadAttr_t UART_RX_Task_attributes = {
  .name = "UART_RX_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for BT_RX_Task — receives velocity commands from ESP32 via USART2 */
osThreadId_t BT_RX_TaskHandle;
const osThreadAttr_t BT_RX_Task_attributes = {
  .name = "BT_RX_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UART_TX_Task */
osThreadId_t UART_TX_TaskHandle;
const osThreadAttr_t UART_TX_Task_attributes = {
  .name = "UART_TX_Task",
  /* Bumped from 1024*4 to absorb the larger ODriveTelemetryMsg local
   * (OdomData grew by ~640 bytes for EKF covariance) plus the extended
   * slim/fat printf varargs. */
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ODriveTask */
osThreadId_t ODriveTaskHandle;
const osThreadAttr_t ODriveTask_attributes = {
  .name = "ODriveTask",
  /* Bumped from 1024*4: holds the EKF instance (~424 B), the larger
   * ODriveTelemetryMsg local (now ~940 B), and the EKF correction call
   * frames. The static scratch in ekf.c keeps the per-call frame small,
   * but this margin makes stack-overflow during heavy SDO bursts a
   * non-issue. */
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IMU_Task (BNO085 SH2 service loop) */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MutexUART_Data */
osMutexId_t MutexUART_DataHandle;
const osMutexAttr_t MutexUART_Data_attributes = {
  .name = "MutexUART_Data"
};
/* USER CODE BEGIN PV */

/*
 * BNO085 shared orientation data.
 * Written by IMU_Task via imu_sensor_data_cb(); read by ODriveTask for telemetry.
 * Values are in radians (yaw/pitch/roll from quaternion via q_to_ypr).
 * Each float is 32-bit aligned → individual reads/writes are atomic on Cortex-M7.
 * The ODriveTask converts to degrees before publishing to match the previous
 * BNO055 (which used degrees). Change RAD_TO_DEG multiplier to 1.0 to use radians.
 */
volatile float g_bno085_yaw   = 0.0f;
volatile float g_bno085_pitch = 0.0f;
volatile float g_bno085_roll  = 0.0f;

/*
 * Quaternion from SH2_GAME_ROTATION_VECTOR (6-axis, no magnetometer).
 * Stored as (qx, qy, qz, qw) where qw is the real (scalar) component, matching
 * the ROS sensor_msgs/Imu convention.
 */
volatile float g_bno085_qx = 0.0f;
volatile float g_bno085_qy = 0.0f;
volatile float g_bno085_qz = 0.0f;
volatile float g_bno085_qw = 1.0f;

/* Angular velocity from SH2_GYROSCOPE_CALIBRATED, rad/s, body frame. */
volatile float g_bno085_wx = 0.0f;
volatile float g_bno085_wy = 0.0f;
volatile float g_bno085_wz = 0.0f;

/* Linear acceleration from SH2_LINEAR_ACCELERATION, m/s^2, body frame (gravity removed). */
volatile float g_bno085_ax = 0.0f;
volatile float g_bno085_ay = 0.0f;
volatile float g_bno085_az = 0.0f;

/* Bumped by IMU_Task whenever a fresh SH2_GAME_ROTATION_VECTOR sample arrives,
 * so the EKF in ODriveTask can apply each BNO085 reading at most once. Wrap on
 * overflow is harmless: the EKF only checks for inequality, not ordering. */
volatile uint32_t g_bno085_seq = 0;

/* Hardware e-stop debounce — sample-after-delay approach.
 *
 * Any edge on ESTOP_PIN (rising or falling) sets g_estop_sample_pending and
 * records the tick. The ODrive task waits ESTOP_DEBOUNCE_MS then reads the
 * actual pin level and acts on that — no edge counting, no separate press/
 * release flags. This is immune to contact bounce on both press and release
 * because 250 ms is far beyond any mechanical settle time (~5–20 ms).
 *
 * If another edge arrives before the 250 ms window expires, the tick is
 * refreshed and the window restarts — so rapid toggles don't leave stale
 * state. */
#define ESTOP_DEBOUNCE_MS      90u
#define ESTOP_RELEASE_HOLD_MS  150u  /* button must be released this long before re-arming */
volatile uint8_t  g_estop_sample_pending = 0;
volatile uint32_t g_estop_sample_tick    = 0;
volatile uint8_t  g_estop_active         = 0;

/* IMU trust flag for the EKF — set to 0 to drop the BNO085 corrections at run
 * time (the filter then runs as a wheel-only kinematic EKF, behaviour close to
 * the old dead-reckoning but with proper covariance). Default: trust the IMU. */
#ifndef EKF_TRUST_IMU
#define EKF_TRUST_IMU 1
#endif

/* Set to 1 by the async event callback when a BNO085 reset event arrives */
static volatile uint8_t g_bno085_sensor_ready = 0;

#define BNO085_REPORT_INTERVAL_US  20000U   /* 50 Hz rotation vector / gyro / linear-accel */
#define RAD_TO_DEG_D               (180.0 / 3.14159265358979323846)

osMessageQueueId_t UART_QueueHandle;
const osMessageQueueAttr_t UART_Queue_attributes = {
  .name = "UARTTask_Queue"
};

osMessageQueueId_t CtrlTsk_QueueHandle;
const osMessageQueueAttr_t CtrlTsk_Queue_attributes = {
  .name = "ControlTask_Queue"
};

osMessageQueueId_t UART2CtrlTsk_QueueHandle;
const osMessageQueueAttr_t UART2CtrlTsk_Queue_attributes = {
  .name = "UART2ControlTask_Queue"
};

osMessageQueueId_t UART2KPIDs_QueueHandle;
const osMessageQueueAttr_t UART2KPIDs_Queue_attributes = {
  .name = "UART2KPIDs_Queue"
};

osMessageQueueId_t kpids_UART_TX_QueueHandle;
const osMessageQueueAttr_t kpids_UART_TX_Queue_attributes = {
  .name = "kpids_UART_TX_Queue"
};

osMessageQueueId_t CAN_2_UTX_QueueHandle;
const osMessageQueueAttr_t CAN_2_UTX_Queue_attributes = {
  .name = "CAN_2_UTX_Queue"
};

osMessageQueueId_t URX_2_CAN_QueueHandle;
const osMessageQueueAttr_t URX_2_CAN_Queue_attributes = {
  .name = "URX_2_CAN_Queue"
};

/* Firmware error queue — depth 16, drained by TX task, written by any task. */
osMessageQueueId_t ERR_QueueHandle;
static const osMessageQueueAttr_t ERR_Queue_attributes = { .name = "ERR_Queue" };
volatile uint8_t g_errors_lost = 0;           /* lost-error counter (non-atomic, diagnostic only) */
volatile uint8_t g_stack_overflow_detected = 0; /* set by vApplicationStackOverflowHook */

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99};
uint8_t RxData[8];

#define ODRIVE_COUNT 4
Axis odrives[ODRIVE_COUNT] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPIO_Estop_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void start_UART_RX_Task(void *argument);
void UART_RX_ParseLine(const char *line_buf, ODriveCmdMsg *odrive_cmd,
                       osMessageQueueId_t UART_QueueHandle,
                       osMessageQueueId_t URX_2_CAN_QueueHandle);
void Start_UART_TX_Task(void *argument);
void StartControlTask(void *argument);
void StartODriveTask(void *argument);
void start_BT_RX_Task(void *argument);  /* ESP32 Bluetooth UART receiver */
void StartIMUTask(void *argument);      /* BNO085 SH2 service loop        */

/* USER CODE BEGIN PFP */
static void FirmwareError_Push(uint8_t code, uint8_t axis, uint8_t detail);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t computeDeltaCNT(uint16_t current, uint16_t previous) {
	int16_t delta = (int16_t)(current - previous);
	return delta;
}

void setMotorDirection(GPIO_TypeDef *port, uint16_t pin1, uint16_t pin2, uint8_t dir) {
	if (dir < 2) {
		dir ? (port->ODR |= (1 << pin1)) : (port->ODR &= ~(1 << pin1));
		!dir ? (port->ODR |= (1 << pin2)) : (port->ODR &= ~(1 << pin2));
	} else {
		port->ODR &= ~(1 << pin1);
		port->ODR &= ~(1 << pin2);
	}
}

int computeNecessaryWheelSpeedsMecanum(double phi, double x_off, double y_off, double r, double u[4], double phi_dot, double y_dot, double x_dot) {
	u[0] = (x_dot*(cos(phi) + sin(phi)) - y_dot*(cos(phi) - sin(phi)) - phi_dot*(x_off + y_off)) / r;
    u[1] = (x_dot*(cos(phi) - sin(phi)) + y_dot*(cos(phi) + sin(phi)) + phi_dot*(x_off + y_off)) / r;
    u[2] = (x_dot*(cos(phi) - sin(phi)) + y_dot*(cos(phi) + sin(phi)) - phi_dot*(x_off + y_off)) / r;
    u[3] = (x_dot*(cos(phi) + sin(phi)) - y_dot*(cos(phi) - sin(phi)) + phi_dot*(x_off + y_off)) / r;
    return 0;
}

int globalSpeedsFromUMecanum(double phi, double x_off, double y_off, double r, double u[4], double q_dot[3]) {
    /* Forward kinematics for a 4-wheel mecanum drive.
     *
     * Step 1: recover body-frame velocities (vx_body, vy_body, phi_dot) from
     * the four wheel angular velocities u[i]. The wheel-sign patterns match
     * those used by computeNecessaryWheelSpeedsMecanum so IK and FK round-trip:
     *     +x body  →  ( +, +, +, + )      = u[0]+u[1]+u[2]+u[3]
     *     +y body  →  ( −, +, +, − )      = -u[0]+u[1]+u[2]-u[3]
     *     +phi     →  ( −, +, −, + )      = -u[0]+u[1]-u[2]+u[3]
     *
     * Step 2: rotate body-frame linear velocities into the world frame using
     * the current heading phi:
     *     x_dot_world = cos(phi)*vx_body − sin(phi)*vy_body
     *     y_dot_world = sin(phi)*vx_body + cos(phi)*vy_body
     *
     * The previous implementation used the y-pattern in q_dot[0] and the
     * phi-pattern in q_dot[2] (rows swapped), so sideways motion appeared as
     * yaw rate / 0 instead of vy, and pure rotation appeared as fake vy.
     */
    const double L = x_off + y_off;
    const double vx_body = (r / 4.0) * ( u[0] + u[1] + u[2] + u[3]);
    const double vy_body = (r / 4.0) * (-u[0] + u[1] + u[2] - u[3]);
    const double phi_dot = (r / (4.0 * L)) * (-u[0] + u[1] - u[2] + u[3]);

    q_dot[0] = phi_dot;
    q_dot[1] = cos(phi) * vx_body - sin(phi) * vy_body;
    q_dot[2] = sin(phi) * vx_body + cos(phi) * vy_body;
    return 0;
}

/* Body-frame mecanum FK: returns (vx_body, vy_body, phi_dot) only — i.e.
 * Step 1 of globalSpeedsFromUMecanum() without the heading rotation. The EKF
 * carries its own theta and rotates the prediction internally, so it needs
 * the body-frame twist as the measurement, not the world-frame one. */
void bodySpeedsFromUMecanum(double x_off, double y_off, double r,
                            const double u[4],
                            double *vx_body, double *vy_body, double *phi_dot)
{
    const double L = x_off + y_off;
    *vx_body = (r / 4.0) * ( u[0] + u[1] + u[2] + u[3]);
    *vy_body = (r / 4.0) * (-u[0] + u[1] + u[2] - u[3]);
    *phi_dot = (r / (4.0 * L)) * (-u[0] + u[1] - u[2] + u[3]);
}

volatile uint8_t BT_active = 0;

/*
 * Tick (HAL_GetTick / osKernelGetTickCount, milliseconds) of the most recent
 * *valid* message received from the ESP32 on USART2. "Valid" means a Type-3
 * line that parsed cleanly into the (vx, vy, wz, buttons) tuple — heartbeat
 * lines are valid too. UINT32_MAX is the sentinel meaning "never received".
 *
 * Written only by start_BT_RX_Task; read by Start_UART_TX_Task to compute
 * the ESP32_age_ms telemetry field. uint32_t reads/writes are atomic on
 * Cortex-M7 (single aligned access), so no mutex is needed.
 */
volatile uint32_t g_bt_last_valid_msg_tick = UINT32_MAX;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  setvbuf(stdin, NULL, _IONBF, 0);
  /* USER CODE END Init */

  SystemClock_Config();
  PeriphCommonClock_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_2 */
__HAL_RCC_HSEM_CLK_ENABLE();
HAL_HSEM_FastTake(HSEM_ID_0);
HAL_HSEM_Release(HSEM_ID_0,0);
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_GPIO_Estop_Init();   /* hardware e-stop input on PE2 — see ESTOP_PIN in main.h */
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
//  MX_SPI1_Init();   /* SPI1 unused — BNO085 is on I2C1 */
  MX_I2C1_Init();   /* BNO085 uses I2C1 (PB6=SCL, PB7=SDA, addr 0x4A) */
  /* Assign ODrive CAN node IDs BEFORE FDCAN RX is activated, so heartbeats are
   * matched from the very first frame (the RX ISR filters by odrives[].NODE_ID,
   * which is otherwise 0 until StartODriveTask runs after the scheduler starts).
   * These must match axis0.config.can.node_id on each S1. */
  odrives[0].NODE_ID = 36;
  odrives[1].NODE_ID = 34;
  odrives[2].NODE_ID = 33;
  odrives[3].NODE_ID = 40;
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t allOK = 1;
  if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) {
  	  printf("tim1 fail\n\r");
  	  allOK = 0;
  }
  if (HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
	  printf("tim2 fail\n\r");
	  allOK = 0;
  }
  if (HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
	  printf("tim4 fail\n\r");
	  allOK = 0;
  }
  if (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) {
	  printf("tim8 fail\n\r");
	  allOK = 0;
  }

  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
  {
  	  printf("tim5 fail\r\n");
  	  allOK = 0;
  	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK)
  {
  	  printf("tim12 fail\r\n");
  	  allOK = 0;
  	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK)
  {
      printf("tim14 fail\r\n");
      allOK = 0;
  	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1) != HAL_OK)
  {
  	  printf("tim3 fail\r\n");
  	  allOK = 0;
  	  Error_Handler();
  }

  if (!allOK) {
	  printf(" El diablo \r\n\n\n\n\n\n\n");
  } else {
	  printf("TODO BN\r\n");
  }

  printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n\n\n\n\r\n");

  /* USER CODE END 2 */

  osKernelInitialize();
  MutexUART_DataHandle = osMutexNew(&MutexUART_Data_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  UART_QueueHandle      = osMessageQueueNew(3,  sizeof(ODriveCmdMsg),      &UART_Queue_attributes);
  CAN_2_UTX_QueueHandle = osMessageQueueNew(3,  sizeof(ODriveTelemetryMsg), &CAN_2_UTX_Queue_attributes);
  URX_2_CAN_QueueHandle = osMessageQueueNew(3,  sizeof(ODriveCmdMsg),      &URX_2_CAN_Queue_attributes);
  ERR_QueueHandle       = osMessageQueueNew(16, sizeof(FirmwareError),     &ERR_Queue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* creation of UART_RX_Task */
  UART_RX_TaskHandle = osThreadNew(start_UART_RX_Task, NULL, &UART_RX_Task_attributes);
  if (UART_RX_TaskHandle == NULL) printf("UART_RX_Task creation FAILED\r\n");

  /* creation of BT_RX_Task — ESP32 Bluetooth commands on USART2 */
  BT_RX_TaskHandle = osThreadNew(start_BT_RX_Task, NULL, &BT_RX_Task_attributes);
  if (BT_RX_TaskHandle == NULL) printf("BT_RX_Task creation FAILED\r\n");

  /* creation of UART_TX_Task */
  UART_TX_TaskHandle = osThreadNew(Start_UART_TX_Task, NULL, &UART_TX_Task_attributes);
  if (UART_TX_TaskHandle == NULL) printf("UART_TX_Task creation FAILED\r\n");

  /* creation of ODriveTask */
  ODriveTaskHandle = osThreadNew(StartODriveTask, NULL, &ODriveTask_attributes);
  if (ODriveTaskHandle == NULL) printf("ODriveTask creation FAILED\r\n");

  /* USER CODE BEGIN RTOS_THREADS */
  /* BNO085 IMU service task — initialises sensor and calls sh2_service() continuously */
  IMU_TaskHandle = osThreadNew(StartIMUTask, NULL, &IMU_Task_attributes);
  if (IMU_TaskHandle == NULL) printf("IMU_Task creation FAILED\r\n");
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  osKernelStart();

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 24;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief Peripherals Common Clock Configuration
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief FDCAN1 Initialization Function
  * CAN_STUB=1 → INTERNAL_LOOPBACK (no external ACK needed, TX never blocks).
  * CAN_STUB=0 → NORMAL mode for real ODrives.
  */
static void MX_FDCAN1_Init(void)
{
  /* USER CODE BEGIN FDCAN1_Init 0 */
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;

#if CAN_STUB
  hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  hfdcan1.Init.AutoRetransmission = DISABLE;
#else
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  /* One-shot transmit (no auto-retransmit). For 100 Hz periodic setpoints a
   * frame that is not ACKed (a momentarily absent ODrive / bus glitch) should
   * be DROPPED and superseded by the next command, NOT retried forever. With
   * AutoRetransmission ENABLED a single un-ACKed frame pins the (small) TX FIFO
   * and rapidly drives the Transmit-Error-Counter to BUS-OFF. */
  hfdcan1.Init.AutoRetransmission = DISABLE;
#endif

  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 31;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;  /* 4 ODrives x (heartbeat+encoder) burst between IRQs */
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;  /* don't wedge TX on a single stalled frame */
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) { Error_Handler(); }

  /* USER CODE BEGIN FDCAN1_Init 2 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_REJECT_REMOTE,
    FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) { Error_Handler(); }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    printf("start fail pipipipipi \n\r");
  } else {
    printf("start succ \n\r");
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {}

  /* STM32H7 FDCAN does NOT auto-recover from bus-off (it sets CCCR.INIT itself
   * and halts). Enable the bus-off notification so HAL_FDCAN_ErrorStatusCallback
   * can flag it; the control task then restarts the peripheral. */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);

  TxHeader.Identifier = 0x42E;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;
  /* USER CODE END FDCAN1_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;   /* BNO085 requires Mode 3 */
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;  /* CS managed in software */
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM4 Initialization Function
  */
static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM5 Initialization Function
  */
static void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 239;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim5);
}

/**
  * @brief TIM8 Initialization Function
  */
static void MX_TIM8_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM12 Initialization Function
  */
static void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 239;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim12);
}

/**
  * @brief TIM13 Initialization Function
  */
static void MX_TIM13_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 239;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim13);
}

/**
  * @brief TIM14 Initialization Function
  */
static void MX_TIM14_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 239;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 19999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim14);
}

/**
  * @brief TIM15 Initialization Function
  */
static void MX_TIM15_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 239;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim15);
}

/**
  * @brief USART2 Initialization Function  (ESP32 Bluetooth link)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USART3 Initialization Function  (ST-Link / ROS debug)
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;   /* must match odrive_dashboard.py baud_rate */
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* BNO085 CS — PB4, output push-pull, idle HIGH (deasserted) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = GPIO_PIN_4;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* BNO085 RST — PD15, output push-pull, idle HIGH (sensor not in reset) */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* BNO085 INT — PD14, input with pull-up (sensor drives LOW when data ready) */
  GPIO_InitStruct.Pin   = GPIO_PIN_14;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* BNO085 WAKE/PS0 — PA4, output push-pull, idle HIGH */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = GPIO_PIN_4;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
  (void)GPIO_InitStruct;
}

/**
  * @brief Init the hardware e-stop input pin (PE2) and its EXTI line.
  *
  * Internal pull-up + rising/falling edge trigger. ISR priority 5 is at
  * the FreeRTOS configMAX_SYSCALL_INTERRUPT_PRIORITY boundary; safe even
  * if a future change makes the ISR call *FromISR APIs. Today the ISR
  * only writes volatile flags so even priority 0 would be technically OK.
  */
static void MX_GPIO_Estop_Init(void)
{
  __HAL_RCC_GPIOE_CLK_ENABLE();      /* idempotent — MX_GPIO_Init already did it */

  GPIO_InitTypeDef gi = {0};

  /* PE2 — ESTOP input, NC button to GND, internal pull-up, both edges. */
  gi.Pin   = ESTOP_PIN;
  gi.Mode  = GPIO_MODE_IT_RISING_FALLING;
  gi.Pull  = GPIO_PULLUP;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESTOP_PORT, &gi);

  /* PE0 — ESTOP LED output, push-pull, starts LOW (off). */
  gi.Pin   = ESTOP_LED_PIN;
  gi.Mode  = GPIO_MODE_OUTPUT_PP;
  gi.Pull  = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESTOP_LED_PORT, &gi);
  HAL_GPIO_WritePin(ESTOP_LED_PORT, ESTOP_LED_PIN, GPIO_PIN_RESET);

  HAL_NVIC_SetPriority(ESTOP_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ESTOP_EXTI_IRQn);
}

/**
  * @brief HAL EXTI callback — any edge on ESTOP_PIN starts the debounce window.
  *
  * Does NOT decide press vs. release here. Simply records that something
  * changed and when. ODriveTask samples the pin after ESTOP_DEBOUNCE_MS and
  * acts on the actual level — immune to contact bounce on both edges.
  * If another edge arrives before the window expires the tick is refreshed.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ESTOP_PIN) {
    g_estop_sample_tick    = HAL_GetTick();
    g_estop_sample_pending = 1;
  }
}

/* USER CODE BEGIN 4 */

static Axis* Find_ODrive_By_NodeID(uint8_t node_id)
{
    for (int i = 0; i < ODRIVE_COUNT; i++)
    {
        if (odrives[i].NODE_ID == node_id)
            return &odrives[i];
    }
    return NULL;
}

/* Set by HAL_FDCAN_ErrorStatusCallback (bit 0 = bus-off) or RxFifo0Callback
 * (bit 1 = RX read fail). Cleared in task context by FDCAN_RecoverIfBusOff(). */
volatile uint8_t g_can_busoff = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        /* Drain ALL pending frames. Four ODrives broadcasting heartbeat +
         * encoder estimates can queue several frames between interrupts; reading
         * only one per IRQ lets the RX FIFO overflow and silently drop telemetry
         * (so axis state/heartbeat appear stale and odometry freezes). */
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
        {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
                g_can_busoff |= 0x02;   /* low bit = bus-off, bit1 = RX read fail; checked in task context */
                break;
            }
            uint8_t node_id = (uint8_t)(RxHeader.Identifier >> 5);
            Axis *target_axis = Find_ODrive_By_NodeID(node_id);
            if (target_axis != NULL)
                ODrive_RX_CallBack(target_axis, &RxHeader, RxData);
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)hfdcan;
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
        g_can_busoff |= 0x01;  /* bit0=bus-off, bit1=RX-read-fail; cleared in task context */
}

/* If FDCAN is bus-off, restart it. STM32H7 FDCAN sets CCCR.INIT itself on
 * bus-off and does NOT self-recover; HAL_FDCAN_Stop() (State->READY) then
 * HAL_FDCAN_Start() clears INIT and begins the 129x11-recessive-bit recovery.
 * Returns 1 if a recovery was performed. Safe to call every control loop. */
uint8_t FDCAN_RecoverIfBusOff(void)
{
    /* Check and clear the RX-read-fail flag set by the ISR (bit 1). */
    if (g_can_busoff & 0x02) {
        g_can_busoff &= ~0x02;
        FirmwareError_Push(FERR_CAN_RX_FAIL, FERR_NO_AXIS, 0);
    }

    FDCAN_ProtocolStatusTypeDef ps = {0};
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    if (g_can_busoff & 0x01 || ps.BusOff)
    {
        HAL_FDCAN_Stop(&hfdcan1);
        HAL_FDCAN_Start(&hfdcan1);
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
        g_can_busoff &= ~0x01;
        FirmwareError_Push(FERR_CAN_BUSOFF, FERR_NO_AXIS, 0);
        return 1;
    }
    return 0;
}

/* Push one FirmwareError into the error queue. If full, increment lost counter. */
static void FirmwareError_Push(uint8_t code, uint8_t axis, uint8_t detail)
{
    FirmwareError err = {code, axis, detail};
    if (osMessageQueuePut(ERR_QueueHandle, &err, 0, 0) != osOK)
        g_errors_lost++;
}

/* Stack overflow hook — called at context-switch if a task's stack guard pattern
 * is corrupted. Cannot use any FreeRTOS API here; drive the ESTOP LED directly
 * as a visual indicator and record the flag for TX task reporting if it survives. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    g_stack_overflow_detected = 1;
    HAL_GPIO_WritePin(ESTOP_LED_PORT, ESTOP_LED_PIN, GPIO_PIN_SET);
    for (;;);   /* system is corrupted — halt */
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    for (;;) osDelay(1000);   /* dead legacy task — kept to avoid CubeMX regen conflicts */
  /* USER CODE END 5 */
}

/* ── UART ring buffers ──────────────────────────────────────────────────── */

/* USER CODE BEGIN Header_start_UART_RX_Task */
#define RX_BUF_SIZE 256
static uint8_t          rx_buf[RX_BUF_SIZE];
static volatile size_t  rx_head = 0;
static volatile size_t  rx_tail = 0;
static uint8_t          rx_char;

/* Second ring buffer for USART2 (ESP32 / Bluetooth) */
#define RX_BUF2_SIZE 256
static uint8_t          rx_buf2[RX_BUF2_SIZE];
static volatile size_t  rx_head2 = 0;
static volatile size_t  rx_tail2 = 0;
static uint8_t          rx_char2;

/* Dead-zone thresholds: commands below these magnitudes are treated as zero */
#define BT_DEADZONE_LINEAR  0.05f
#define BT_DEADZONE_ANGULAR 0.05f

/**
 * @brief UART RX interrupt callback — handles both USART3 (ROS/debug) and
 *        USART2 (ESP32 Bluetooth).
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        rx_buf[rx_tail] = rx_char;
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
        HAL_UART_Receive_IT(huart, &rx_char, 1);
    } else if (huart->Instance == USART2) {
        rx_buf2[rx_tail2] = rx_char2;
        rx_tail2 = (rx_tail2 + 1) % RX_BUF2_SIZE;
        HAL_UART_Receive_IT(huart, &rx_char2, 1);
    }
}

/**
 * @brief  Re-arm interrupt RX after ANY UART error.
 *
 * On an overrun (ORE) — common at startup, under load, or with line noise — the
 * STM32 HAL tears down interrupt reception (RxState->READY, RxISR=NULL) and
 * calls this (otherwise-empty weak) callback WITHOUT re-arming. Since RX is only
 * re-armed in HAL_UART_RxCpltCallback (which never runs on the error path), the
 * link would die permanently until reset. Clearing the sticky error flags and
 * re-issuing HAL_UART_Receive_IT() makes both links self-heal.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    if (huart->Instance == USART3) {
        HAL_UART_Receive_IT(huart, &rx_char, 1);    /* ROS / debug link */
    } else if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(huart, &rx_char2, 1);   /* ESP32 / PS5 link */
    }
}

/* USER CODE END Header_start_UART_RX_Task */
void start_UART_RX_Task(void *argument)
{
  /* USER CODE BEGIN start_UART_RX_Task */
    char line_buf[RX_BUF_SIZE] = {0};
    uint16_t line_index = 0;
    ODriveCmdMsg odrive_cmd = {0};

    HAL_UART_Receive_IT(&huart3, &rx_char, 1);

    for (;;) {
        while (rx_head != rx_tail) {
            uint8_t byte = rx_buf[rx_head];
            rx_head = (rx_head + 1) % RX_BUF_SIZE;

            if (byte == '\n' || byte == '\r') {
                if (line_index > 0) {
                    line_buf[line_index] = '\0';
                    UART_RX_ParseLine(line_buf, &odrive_cmd, UART_QueueHandle, URX_2_CAN_QueueHandle);
                }
                line_index = 0;
                memset(line_buf, 0, sizeof(line_buf));
            } else if (line_index < RX_BUF_SIZE - 1) {
                line_buf[line_index++] = byte;
            } else {
                line_index = 0;
                memset(line_buf, 0, sizeof(line_buf));
            }
        }
        osDelay(5);
    }
  /* USER CODE END start_UART_RX_Task */
}

/* USER CODE BEGIN Header_start_BT_RX_Task */
/* USER CODE END Header_start_BT_RX_Task */
/**
 * @brief Receives velocity commands from the ESP32 via USART2 (Bluetooth).
 *
 * Protocol: "3 <vx> <vy> <wz> <buttons_hex>\r\n"
 *   vx, vy, wz in m/s and rad/s (robot frame).
 *   buttons_hex: hex bitmask — bit 3 is emergency stop.
 *
 * Dead-zones are applied before the command is queued so that small joystick
 * noise does not produce creep motion.
 */
void start_BT_RX_Task(void *argument)
{
  /* USER CODE BEGIN start_BT_RX_Task */
    char     line_buf2[RX_BUF2_SIZE] = {0};
    uint16_t line_index2 = 0;
    osStatus_t qst;

    /* Watchdog: print "no data" every 2 s if nothing arrives on USART2 */
    uint32_t last_rx_tick  = osKernelGetTickCount();
    uint32_t total_bytes   = 0;
    uint32_t BT_WATCHDOG_MS = 1000;

    HAL_UART_Receive_IT(&huart2, &rx_char2, 1);

    for (;;) {
        uint8_t got_any = 0;

        while (rx_head2 != rx_tail2) {
            uint8_t byte = rx_buf2[rx_head2];
            rx_head2 = (rx_head2 + 1) % RX_BUF2_SIZE;
            got_any = 1;
            total_bytes++;

            if (byte == '\n' || byte == '\r') {
                if (line_index2 > 0) {
                    line_buf2[line_index2] = '\0';
//                    printf("BT RX raw: \"%s\"\r\n", line_buf2);

                    int msg_type = 0;
                    if (sscanf(line_buf2, "%d", &msg_type) == 1 && msg_type == 3) {
                        float vx = 0.0f, vy = 0.0f, wz = 0.0f;
                        unsigned int buttons_u = 0;
                        unsigned int controll_state_u = 0;
                        int parsed = sscanf(line_buf2, "%*d %f %f %f %x %u",
                                            &vx, &vy, &wz, &buttons_u,
                                            &controll_state_u);
                        if (parsed >= 4) {
                            /* Mark this as a valid ESP32 message for the
                             * connection-age telemetry. We update on every
                             * successful Type-3 parse (including paired/
                             * inactive heartbeats), since any such packet
                             * proves the ESP32→STM32 link is alive. */
                            g_bt_last_valid_msg_tick = HAL_GetTick();

                            /*
                             * controll_state mirrors the ESP32's full status:
                             *   0 = no controller paired        → BT_active 0 (Inactive)
                             *   1 = paired, TX disabled         → BT_active 1 (Paired)
                             *   2 = paired, TX enabled          → BT_active 2 (Active)
                             *
                             * Fall back to 1 if the field is missing (old firmware).
                             */
                            uint8_t controll_state = (parsed >= 5)
                                                     ? (uint8_t)(controll_state_u <= 2
                                                                  ? controll_state_u : 2)
                                                     : 1;

                            /* Directly map ESP32 state to BT_active */
                            if (controll_state != BT_active) {
                                uint8_t prev = BT_active;
                                BT_active = controll_state;

                                /*
                                 * Transitioning OUT of Active: send one stop command so
                                 * the state machine releases BT override immediately
                                 * instead of waiting for the watchdog timeout.
                                 */
                                if (prev == 2 && BT_active < 2) {
                                    ODriveCmdMsg stop_cmd = {0};
                                    stop_cmd.type           = ODRIVE_CMD_SET_VEL;
                                    stop_cmd.target_mask    = 0x0F;
                                    stop_cmd.source         = CMD_SOURCE_BT;
                                    stop_cmd.robot_twist[0] = 0.0f;
                                    stop_cmd.robot_twist[1] = 0.0f;
                                    stop_cmd.robot_twist[2] = 0.0f;

                                    qst = osMessageQueuePut(URX_2_CAN_QueueHandle,
                                                            &stop_cmd, 0, 0);
                                    if (qst != osOK)
                                        FirmwareError_Push(FERR_BT_QUEUE_FULL, FERR_NO_AXIS, 0);
                                }
                            }

                            /* Only queue motion commands when Active */
                            if (BT_active != 2) {
                                line_index2 = 0;
                                memset(line_buf2, 0, sizeof(line_buf2));
                                continue;
                            }

                            if (vx > -BT_DEADZONE_LINEAR  && vx < BT_DEADZONE_LINEAR)  vx = 0.0f;
                            if (vy > -BT_DEADZONE_LINEAR  && vy < BT_DEADZONE_LINEAR)  vy = 0.0f;
                            if (wz > -BT_DEADZONE_ANGULAR && wz < BT_DEADZONE_ANGULAR) wz = 0.0f;

                            uint16_t buttons = (uint16_t)(buttons_u & 0xFFFF);

                            ODriveCmdMsg bt_cmd = {0};
                            bt_cmd.type           = ODRIVE_CMD_SET_VEL;
                            bt_cmd.target_mask    = 0x0F;
                            bt_cmd.source         = CMD_SOURCE_BT;
                            bt_cmd.buttons        = buttons;
                            bt_cmd.robot_twist[0] = vx;
                            bt_cmd.robot_twist[1] = vy;
                            bt_cmd.robot_twist[2] = wz;

                            qst = osMessageQueuePut(URX_2_CAN_QueueHandle, &bt_cmd, 0, 0);
                            if (qst != osOK)
                                FirmwareError_Push(FERR_BT_QUEUE_FULL, FERR_NO_AXIS, 1);
                        } else {
                            FirmwareError_Push(FERR_BT_PARSE_FAIL, FERR_NO_AXIS, 0);
                        }
                    }
                }
                line_index2 = 0;
                memset(line_buf2, 0, sizeof(line_buf2));
            } else if (line_index2 < RX_BUF2_SIZE - 1) {
                line_buf2[line_index2++] = byte;
            } else {
                line_index2 = 0;
                memset(line_buf2, 0, sizeof(line_buf2));
                FirmwareError_Push(FERR_BT_QUEUE_FULL, FERR_NO_AXIS, 2);
            }
        }

        if (got_any) {
            last_rx_tick = osKernelGetTickCount();
        } else if ((osKernelGetTickCount() - last_rx_tick) >= BT_WATCHDOG_MS) {
            last_rx_tick = osKernelGetTickCount();
            /* Self-heal: if interrupt RX was torn down by an error and the
             * ErrorCallback somehow did not re-arm it, restart it here. */
            if (huart2.RxState == HAL_UART_STATE_READY) {
                __HAL_UART_CLEAR_OREFLAG(&huart2);
                HAL_UART_Receive_IT(&huart2, &rx_char2, 1);
            }
        }

        osDelay(5);
    }
  /* USER CODE END start_BT_RX_Task */
}

/* USER CODE BEGIN Header_Start_UART_TX_Task */
/* USER CODE END Header_Start_UART_TX_Task */
void Start_UART_TX_Task(void *argument)
{
  /* USER CODE BEGIN Start_UART_TX_Task */
    ODriveCmdMsg last_cmd = {0};
    ODriveTelemetryMsg telemetryMsg = {0};
    osStatus_t qst1, qst2;
    uint32_t tx_count = 0;   /* throttles the fat diagnostic line vs the slim high-rate line */

    for (;;)
    {
        qst1 = osMessageQueueGet(CAN_2_UTX_QueueHandle, &telemetryMsg, NULL, osWaitForever);
        qst2 = osMessageQueueGet(UART_QueueHandle, &last_cmd, NULL, 0);
        (void)qst1; (void)qst2;

        /* Always reflect the live BT_active — avoids stale values between SM updates */
        telemetryMsg.bt_active = BT_active;

        /* Connection-age for the ESP32→STM32 Bluetooth link.
         * Snapshot the volatile tick once to avoid a torn read if the BT task
         * updates it mid-printf. -1 (cast to long) signals "no valid ESP32
         * message has been received since boot" on the ROS side. */
        uint32_t bt_tick_snap = g_bt_last_valid_msg_tick;
        long esp32_age_ms;
        if (bt_tick_snap == UINT32_MAX) {
            esp32_age_ms = -1;
        } else {
            esp32_age_ms = (long)(HAL_GetTick() - bt_tick_snap);
        }

        /* ─────────────────────────────────────────────────────────────────
         *  TELEMETRY FIELD INDEX MAP — shared by the slim and fat printfs.
         *  Each field on the wire is `i=value`. The dashboard's TELEM_FIELDS
         *  list (odrive_comm/odrive_dashboard.py) must match this table
         *  EXACTLY — adding/reordering a field requires editing both in
         *  lock-step plus a `colcon build` of `odrive_comm` on the ROS side.
         *
         *  Slim line (every cycle, ~50 Hz): emits indices 3, 12, 20..38
         *     — the 21 EKF + raw-IMU fields the dashboard turns into the
         *     /odrive/imu + /odrive/odom messages.
         *  Fat  line (every 5th cycle, ~10 Hz): emits indices 0..95 — full
         *     diagnostic snapshot. Slim's fields are duplicated here so a
         *     dropped slim line doesn't starve the dashboard.
         *
         *      0    CMD_vx           commanded body-x velocity     [m/s]
         *      1    CMD_vy           commanded body-y velocity     [m/s]
         *      2    CMD_wz           commanded yaw rate            [rad/s]
         *      3    IMU_yaw          BNO085 yaw                    [deg]
         *      4    IMU_roll         BNO085 roll                   [deg]
         *      5    IMU_pitch        BNO085 pitch                  [deg]
         *      6..9  IMU_qx/qy/qz/qw orientation quaternion        [-]
         *      10..12 IMU_wx/wy/wz   angular velocity              [rad/s]
         *      13..15 IMU_ax/ay/az   linear acceleration           [m/s^2]
         *      16..19 IK_u0..u3      IK-computed wheel speeds      [rad/s]
         *      20   ODOM_phi         EKF yaw                       [rad]
         *      21   ODOM_x           EKF x (world)                 [m]
         *      22   ODOM_y           EKF y (world)                 [m]
         *      23   ODOM_z           pose z (always 0)             [m]
         *      24..27 ODOM_qx/qy/qz/qw  EKF orientation quat       [-]
         *      28   ODOM_w           EKF yaw rate                  [rad/s]
         *      29   ODOM_vx          world x velocity              [m/s]
         *      30   ODOM_vy          world y velocity              [m/s]
         *      31   ODOM_vxb         body x velocity               [m/s]
         *      32   ODOM_vyb         body y velocity               [m/s]
         *      33   ODOM_var_x       pose-x variance               [m^2]
         *      34   ODOM_var_y       pose-y variance               [m^2]
         *      35   ODOM_var_yaw     yaw variance                  [rad^2]
         *      36   ODOM_var_vx      twist-vx variance             [(m/s)^2]
         *      37   ODOM_var_vy      twist-vy variance             [(m/s)^2]
         *      38   ODOM_var_wz      twist-wz variance             [(rad/s)^2]
         *      39..51 Axis-0 block   N,E,S,C,P,V,Sh,CPR,Vbus,Ibus,IqSet,IqMeas,U
         *      52..64 Axis-1 block   (same 13-field layout)
         *      65..77 Axis-2 block   (same 13-field layout)
         *      78..90 Axis-3 block   (same 13-field layout)
         *      91   BT_active        BT control armed              [bool]
         *      92   BT_vx            BT commanded body-x velocity  [m/s]
         *      93   BT_vy            BT commanded body-y velocity  [m/s]
         *      94   BT_wz            BT commanded yaw rate         [rad/s]
         *      95   ESP32_age_ms     ms since last valid ESP32 msg [ms]
         *      96   SM_state         firmware state machine        [enum]
         *                              0 = SM_BOOT      (waiting to arm)
         *                              1 = SM_STARTUP   (arming axes)
         *                              2 = SM_RUNNING   (normal operation)
         *                              3 = SM_IDLE      (commanded stop)
         *                              4 = SM_ESTOP     (hardware button held)
         * ──────────────────────────────────────────────────────────────── */

        /* SLIM line — sent every cycle. ~225 chars at ~50 Hz ≈ 11 kB/s. */
        printf("3=%.2f,12=%.4f,"
               "20=%.3f,21=%.3f,22=%.3f,23=%.3f,"
               "24=%.5f,25=%.5f,26=%.5f,27=%.5f,"
               "28=%.3f,29=%.3f,30=%.3f,"
               "31=%.3f,32=%.3f,"
               "33=%.5f,34=%.5f,35=%.5f,"
               "36=%.5f,37=%.5f,38=%.5f,"
               "96=%u\r\n",
               telemetryMsg.imu.yaw, telemetryMsg.imu.wz,
               telemetryMsg.odom.phi, telemetryMsg.odom.x_pos, telemetryMsg.odom.y_pos,
               telemetryMsg.odom.z_pos,
               (double)telemetryMsg.odom.qx, (double)telemetryMsg.odom.qy,
               (double)telemetryMsg.odom.qz, (double)telemetryMsg.odom.qw,
               telemetryMsg.odom.q_dot[0], telemetryMsg.odom.q_dot[1], telemetryMsg.odom.q_dot[2],
               telemetryMsg.odom.vx_body,  telemetryMsg.odom.vy_body,
               telemetryMsg.odom.pose_covariance[0 * 6 + 0],
               telemetryMsg.odom.pose_covariance[1 * 6 + 1],
               telemetryMsg.odom.pose_covariance[5 * 6 + 5],
               telemetryMsg.odom.twist_covariance[0 * 6 + 0],
               telemetryMsg.odom.twist_covariance[1 * 6 + 1],
               telemetryMsg.odom.twist_covariance[5 * 6 + 5],
               telemetryMsg.sm_state);

        /* FAT line — every 5th slim cycle. ~990 chars at ~10 Hz ≈ 10 kB/s. */
        if (++tx_count % 5 == 0)
        printf("0=%.3lf,1=%.3lf,2=%.3lf,"
               "3=%.2f,4=%.2f,5=%.2f,"
               "6=%.6f,7=%.6f,8=%.6f,9=%.6f,"
               "10=%.4f,11=%.4f,12=%.4f,"
               "13=%.4f,14=%.4f,15=%.4f,"
               "16=%.3lf,17=%.3lf,18=%.3lf,19=%.3lf,"
               "20=%.3f,21=%.3f,22=%.3f,23=%.3f,"
               "24=%.5f,25=%.5f,26=%.5f,27=%.5f,"
               "28=%.3f,29=%.3f,30=%.3f,"
               "31=%.3f,32=%.3f,"
               "33=%.5f,34=%.5f,35=%.5f,"
               "36=%.5f,37=%.5f,38=%.5f,"
               "39=%u,40=%lu,41=%u,42=%u,43=%.3f,44=%.3f,45=%ld,46=%ld,47=%.3f,48=%.3f,49=%.3f,50=%.3f,51=%u,"
               "52=%u,53=%lu,54=%u,55=%u,56=%.3f,57=%.3f,58=%ld,59=%ld,60=%.3f,61=%.3f,62=%.3f,63=%.3f,64=%u,"
               "65=%u,66=%lu,67=%u,68=%u,69=%.3f,70=%.3f,71=%ld,72=%ld,73=%.3f,74=%.3f,75=%.3f,76=%.3f,77=%u,"
               "78=%u,79=%lu,80=%u,81=%u,82=%.3f,83=%.3f,84=%ld,85=%ld,86=%.3f,87=%.3f,88=%.3f,89=%.3f,90=%u,"
               "91=%u,92=%.3f,93=%.3f,94=%.3f,"
               "95=%ld,96=%u\r\n",
               last_cmd.robot_twist[0], last_cmd.robot_twist[1], last_cmd.robot_twist[2],
               telemetryMsg.imu.yaw, telemetryMsg.imu.roll, telemetryMsg.imu.pitch,
               telemetryMsg.imu.qx, telemetryMsg.imu.qy, telemetryMsg.imu.qz, telemetryMsg.imu.qw,
               telemetryMsg.imu.wx, telemetryMsg.imu.wy, telemetryMsg.imu.wz,
               telemetryMsg.imu.ax, telemetryMsg.imu.ay, telemetryMsg.imu.az,
               telemetryMsg.IK_computed_wheel_speeds[0], telemetryMsg.IK_computed_wheel_speeds[1],
               telemetryMsg.IK_computed_wheel_speeds[2], telemetryMsg.IK_computed_wheel_speeds[3],
               telemetryMsg.odom.phi, telemetryMsg.odom.x_pos, telemetryMsg.odom.y_pos,
               telemetryMsg.odom.z_pos,
               (double)telemetryMsg.odom.qx, (double)telemetryMsg.odom.qy,
               (double)telemetryMsg.odom.qz, (double)telemetryMsg.odom.qw,
               telemetryMsg.odom.q_dot[0], telemetryMsg.odom.q_dot[1], telemetryMsg.odom.q_dot[2],
               telemetryMsg.odom.vx_body, telemetryMsg.odom.vy_body,
               telemetryMsg.odom.pose_covariance[0 * 6 + 0],
               telemetryMsg.odom.pose_covariance[1 * 6 + 1],
               telemetryMsg.odom.pose_covariance[5 * 6 + 5],
               telemetryMsg.odom.twist_covariance[0 * 6 + 0],
               telemetryMsg.odom.twist_covariance[1 * 6 + 1],
               telemetryMsg.odom.twist_covariance[5 * 6 + 5],
               telemetryMsg.node_id[0], telemetryMsg.axis_error[0], telemetryMsg.axis_state[0], telemetryMsg.controller_status[0],
               telemetryMsg.pos_est[0], telemetryMsg.vel_est[0], telemetryMsg.encoder_shadow[0], telemetryMsg.encoder_cpr[0],
               telemetryMsg.bus_voltage[0], telemetryMsg.bus_current[0], telemetryMsg.iq_setpoint[0], telemetryMsg.iq_measured[0], telemetryMsg.updated[0],
               telemetryMsg.node_id[1], telemetryMsg.axis_error[1], telemetryMsg.axis_state[1], telemetryMsg.controller_status[1],
               telemetryMsg.pos_est[1], telemetryMsg.vel_est[1], telemetryMsg.encoder_shadow[1], telemetryMsg.encoder_cpr[1],
               telemetryMsg.bus_voltage[1], telemetryMsg.bus_current[1], telemetryMsg.iq_setpoint[1], telemetryMsg.iq_measured[1], telemetryMsg.updated[1],
               telemetryMsg.node_id[2], telemetryMsg.axis_error[2], telemetryMsg.axis_state[2], telemetryMsg.controller_status[2],
               telemetryMsg.pos_est[2], telemetryMsg.vel_est[2], telemetryMsg.encoder_shadow[2], telemetryMsg.encoder_cpr[2],
               telemetryMsg.bus_voltage[2], telemetryMsg.bus_current[2], telemetryMsg.iq_setpoint[2], telemetryMsg.iq_measured[2], telemetryMsg.updated[2],
               telemetryMsg.node_id[3], telemetryMsg.axis_error[3], telemetryMsg.axis_state[3], telemetryMsg.controller_status[3],
               telemetryMsg.pos_est[3], telemetryMsg.vel_est[3], telemetryMsg.encoder_shadow[3], telemetryMsg.encoder_cpr[3],
               telemetryMsg.bus_voltage[3], telemetryMsg.bus_current[3], telemetryMsg.iq_setpoint[3], telemetryMsg.iq_measured[3], telemetryMsg.updated[3],
               telemetryMsg.bt_active, telemetryMsg.bt_vx, telemetryMsg.bt_vy, telemetryMsg.bt_wz,
               esp32_age_ms,
               telemetryMsg.sm_state);

        /* Drain the firmware error queue — emit each entry before the next
         * telemetry line so host can correlate errors with the preceding state. */
        {
            FirmwareError ferr;
            while (osMessageQueueGet(ERR_QueueHandle, &ferr, NULL, 0) == osOK)
                printf("E=%u,%u,%u\r\n", ferr.code, ferr.axis, ferr.detail);

            if (g_errors_lost > 0) {
                uint8_t lost = g_errors_lost;
                g_errors_lost = 0;
                printf("ELOST=%u\r\n", lost);
            }
        }

        /* Periodic stack watermark check (~5 s interval at 20 ms/cycle).
         * Pushes FERR_STACK_LOW with axis = task index, detail = remaining words. */
        {
            static uint16_t wm_ctr = 0;
            if (++wm_ctr >= 250) {
                wm_ctr = 0;
                struct { osThreadId_t h; uint8_t idx; } tasks[] = {
                    {ODriveTaskHandle,  0},
                    {UART_TX_TaskHandle,1},
                    {IMU_TaskHandle,    2},
                    {BT_RX_TaskHandle,  3},
                    {UART_RX_TaskHandle,4},
                };
                for (uint8_t t = 0; t < 5; t++) {
                    if (tasks[t].h == NULL) continue;
                    UBaseType_t wm = uxTaskGetStackHighWaterMark(tasks[t].h);
                    if (wm < 128)
                        FirmwareError_Push(FERR_STACK_LOW, tasks[t].idx, (uint8_t)(wm > 255 ? 255 : wm));
                }
            }
        }

        osDelay(10);
    }
  /* USER CODE END Start_UART_TX_Task */
}

/* USER CODE BEGIN Header_StartControlTask */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
    for(;;) { osDelay(10); }
  /* USER CODE END StartControlTask */
}

/* -------------------------------------------------------------------------
 * BNO085 IMU Task — SH2 callbacks and service loop
 * ---------------------------------------------------------------------- */

static void imu_async_event_cb(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET) {
        g_bno085_sensor_ready = 1;
    }
}

static void imu_sensor_data_cb(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t val;
    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) return;

    /* Each 32-bit float write is atomic on Cortex-M7 (aligned store).
     * ODriveTask reads these at most once per 10 ms, so the worst case
     * is reading qx from sample N and the rest from sample N+1 —
     * acceptable for robot telemetry at 50 Hz. */
    switch (val.sensorId) {

    case SH2_GAME_ROTATION_VECTOR: {
        /* 6-axis orientation (gyro + accel, NO magnetometer). Chosen over the
         * 9-axis SH2_ROTATION_VECTOR because the ODrive motors' holding current
         * disturbs the magnetometer even at rest, which yanked the fused yaw
         * ("flash"/latigazo) at random. The EKF fuses yaw with imu0_relative, so
         * no absolute-north reference is needed; the only trade-off is slow gyro
         * yaw drift (a few deg/min), which is harmless for a short run and is
         * corrected by SLAM scan-matching during navigation.
         * BNO085 reports (i, j, k, real); ROS expects (x, y, z, w). */
        g_bno085_qx = val.un.gameRotationVector.i;
        g_bno085_qy = val.un.gameRotationVector.j;
        g_bno085_qz = val.un.gameRotationVector.k;
        g_bno085_qw = val.un.gameRotationVector.real;

        float yaw, pitch, roll;
        q_to_ypr(val.un.gameRotationVector.real,
                 val.un.gameRotationVector.i,
                 val.un.gameRotationVector.j,
                 val.un.gameRotationVector.k,
                 &yaw, &pitch, &roll);
        g_bno085_yaw   = yaw;
        g_bno085_pitch = pitch;
        g_bno085_roll  = roll;

        /* Mark a fresh BNO085 sample for the EKF — incremented LAST so the
         * ODriveTask never observes the new seq with a stale yaw. */
        g_bno085_seq++;
        break;
    }

    case SH2_GYROSCOPE_CALIBRATED:
        /* Angular velocity in rad/s, body frame — matches ROS Imu convention. */
        g_bno085_wx = val.un.gyroscope.x;
        g_bno085_wy = val.un.gyroscope.y;
        g_bno085_wz = val.un.gyroscope.z;
        break;

    case SH2_LINEAR_ACCELERATION:
        /* Linear acceleration in m/s^2 with gravity already removed by the
         * BNO085 sensor fusion. ROS Imu wants the IMU's measured acceleration
         * (which normally includes gravity), but using linear-acceleration is
         * a common pragmatic choice for wheeled robots; downstream consumers
         * can integrate this directly. */
        g_bno085_ax = val.un.linearAcceleration.x;
        g_bno085_ay = val.un.linearAcceleration.y;
        g_bno085_az = val.un.linearAcceleration.z;
        break;

    default:
        break;
    }
}

static void imu_service_ms(uint32_t ms)
{
    uint32_t t0 = osKernelGetTickCount();
    while ((osKernelGetTickCount() - t0) < ms) {
        sh2_service();
        osDelay(1);
    }
}

static void imu_enable_report(sh2_SensorId_t sensor_id, const char *name)
{
    (void)name;
    sh2_SensorConfig_t cfg;
    __builtin_memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = BNO085_REPORT_INTERVAL_US;
    int rc = sh2_setSensorConfig(sensor_id, &cfg);
    if (rc != SH2_OK)
        FirmwareError_Push(FERR_IMU_REPORT_CFG, FERR_NO_AXIS, (uint8_t)(rc & 0xFF));
}

static void imu_enable_all_reports(void)
{
    /* Required to populate orientation, angular velocity, and linear
     * acceleration in the ROS sensor_msgs/Imu message. */
    imu_enable_report(SH2_GAME_ROTATION_VECTOR, "GAME_ROTATION_VECTOR");
    imu_enable_report(SH2_GYROSCOPE_CALIBRATED, "GYROSCOPE_CALIBRATED");
    imu_enable_report(SH2_LINEAR_ACCELERATION,  "LINEAR_ACCELERATION");
}

void StartIMUTask(void *argument)
{
#if !IMU_ENABLED
    /* IMU disabled at compile time — skip all SH2/I2C initialisation. */
    for (;;) osDelay(1000);
#else
    osDelay(500);

    int rc = sh2_open(BNO085_GetHal(), imu_async_event_cb, NULL);
    if (rc != SH2_OK) {
        FirmwareError_Push(FERR_IMU_OPEN, FERR_NO_AXIS, (uint8_t)(rc & 0xFF));
        for (;;) osDelay(1000);
    }

    /* Drain startup traffic while letting the SHTP/SH2 layer settle */
    imu_service_ms(200);

    rc = sh2_setSensorCallback(imu_sensor_data_cb, NULL);
    if (rc != SH2_OK)
        FirmwareError_Push(FERR_IMU_SET_CALLBACK, FERR_NO_AXIS, (uint8_t)(rc & 0xFF));

    /* Give SH2 time to process control/startup packets before config */
    imu_service_ms(100);

    imu_enable_all_reports();

    /* Clear any spurious reset flag that arrived during sh2_open startup */
    g_bno085_sensor_ready = 0;

    for (;;) {
        sh2_service();

        /* If the sensor reset (e.g. power glitch), re-enable the rotation vector */
        if (g_bno085_sensor_ready) {
            g_bno085_sensor_ready = 0;
            FirmwareError_Push(FERR_IMU_RESET, FERR_NO_AXIS, 0);
            imu_service_ms(200);
            imu_enable_all_reports();
        }

        /* Yield for ~2 ms. sh2_service() returns immediately when INT is high
         * (no data pending), so this keeps the task at ~500 Hz poll without
         * hammering the I2C bus (I2C only transacts when INT is LOW). */
        osDelay(2);
    }
#endif /* IMU_ENABLED */
}

/* USER CODE BEGIN Header_StartODriveTask */

/* Tuned velocity-loop gains pushed into every ODrive at bring-up so the live
 * runtime values match the snapshots in odrive_config_dump/odrive_node*.json.
 * The ODrives have NOT been save_configuration()'d with these values, so the
 * STM32 re-applies them on each boot via the SET_VEL_GAINS CAN frame — the
 * write is RAM-only on the ODrive side. Update both this define and every
 * odrive_node*.json in lock-step if the tuned values change. */
#define ODRIVE_STARTUP_VEL_GAIN     0.3333f
#define ODRIVE_STARTUP_VEL_INT_GAIN 5.658f

/* axis0.controller.config.vel_ramp_rate, written via RxSdo (Set_Param_Float).
 * Endpoint ID is from flat_endpoints.json for fw v0.6.12 -- re-check this ID
 * if any ODrive is ever reflashed to a different firmware version, since IDs
 * are not guaranteed stable across firmware releases. RAM-only write, same
 * caveat as the gains above: re-applied every boot, not save_configuration()'d. */
#define ODRIVE_VEL_RAMP_RATE_ENDPOINT_ID 398
#define ODRIVE_STARTUP_VEL_RAMP_RATE     40.0f   /* was 25.0 — snappier accel; motor current_soft_max raised to 20A to back it */

HAL_StatusTypeDef ODrive_Startup(Axis odrives[], uint8_t num_odrives, FDCAN_TXmsg *msg,
                                  Control_Mode control_mode, Input_Mode input_mode,
                                  uint8_t requested_state)
{
    HAL_StatusTypeDef st;
    for (uint8_t i = 0; i < num_odrives; i++)
    {
        FDCAN_WAIT_TX_FREE();
        st = Clear_Errors(&odrives[i], msg);
        if (st != HAL_OK) return st;

        FDCAN_WAIT_TX_FREE();
        st = Set_Controller_Modes(&odrives[i], msg, control_mode, input_mode);
        if (st != HAL_OK) return st;

        /* Push the project-tuned velocity-loop gains BEFORE arming, so the
         * controller uses them on its very first SET_INPUT_VEL frame.
         * SET_VEL_GAINS doesn't reset axis state — safe to issue anytime. */
        FDCAN_WAIT_TX_FREE();
        st = Set_Vel_Gains(&odrives[i], msg,
                           ODRIVE_STARTUP_VEL_GAIN, ODRIVE_STARTUP_VEL_INT_GAIN);
        if (st != HAL_OK) return st;

        FDCAN_WAIT_TX_FREE();
        st = Set_Axis_Requested_State(&odrives[i], msg, requested_state);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

/* Re-issues the arm triplet (clear errors, set velocity mode, request
 * CLOSED_LOOP_CONTROL) and waits for the HEARTBEAT to confirm the axis actually
 * reached CLOSED_LOOP_CONTROL — instead of trusting that a queued CAN frame took
 * effect. Returns 1 on confirmed arm, 0 if it never confirmed within `attempts`.
 * The ~10 Hz heartbeat keeps axis->AXIS_Current_State fresh via ODrive_RX_CallBack.
 *
 * CAN_STUB=1: loopback does not fake heartbeat responses, so AXIS_Current_State
 * never updates. Skip the confirmation loop and return success immediately so
 * the state machine transitions normally during bench testing. */
static uint8_t ODrive_ArmAxisConfirmed(Axis *axis, FDCAN_TXmsg *msg,
                                       Control_Mode ctrl, Input_Mode in_mode,
                                       uint8_t attempts, uint8_t axis_idx)
{
#if CAN_STUB
    (void)attempts; (void)axis_idx;
    FDCAN_WAIT_TX_FREE(); Clear_Errors(axis, msg);
    FDCAN_WAIT_TX_FREE(); Set_Controller_Modes(axis, msg, ctrl, in_mode);
    FDCAN_WAIT_TX_FREE(); Set_Axis_Requested_State(axis, msg, CLOSED_LOOP_CONTROL);
    return 1;
#else
    for (uint8_t a = 0; a < attempts; a++) {
        FDCAN_WAIT_TX_FREE(); Clear_Errors(axis, msg);
        FDCAN_WAIT_TX_FREE(); Set_Controller_Modes(axis, msg, ctrl, in_mode);
        FDCAN_WAIT_TX_FREE(); Set_Axis_Requested_State(axis, msg, CLOSED_LOOP_CONTROL);
        for (uint8_t w = 0; w < 30; w++) {          /* wait up to ~300 ms */
            osDelay(10);
            if (axis->AXIS_Current_State == CLOSED_LOOP_CONTROL) return 1;
        }
        (void)a;
    }
    /* axis_idx (0-3), not NODE_ID, so this matches every other error code's
     * axis numbering convention. */
    FirmwareError_Push(FERR_ARM_TIMEOUT, axis_idx, (uint8_t)axis->AXIS_Current_State);
    return 0;
#endif
}


void ODrive_ProcessCommand(const ODriveCmdMsg *cmd, Axis odrives[], uint8_t num_odrives,
                           FDCAN_TXmsg *tx, OdomData *odrive_odom,
                           double x_offset, double y_offset, double radius,
                           Control_Mode *current_ctrl_mode, Input_Mode *current_input_mode,
                           double wheel_sign[], double u[])
{
    HAL_StatusTypeDef st;
    double x_dot = 0.0, y_dot = 0.0, phi_dot = 0.0;

    switch (cmd->type)
    {
        case ODRIVE_CMD_SET_VEL:
        {
            if (*current_ctrl_mode != VELOCITY_CONTROL) break;
            x_dot   = cmd->robot_twist[0];
            y_dot   = cmd->robot_twist[1];
            phi_dot = cmd->robot_twist[2];
            computeNecessaryWheelSpeedsMecanum(0.0, x_offset, y_offset, radius, u, phi_dot, y_dot, x_dot);
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Input_Vel(&odrives[i], tx,
                         wheel_sign[i] * (float)((u[i] * odrives[i].gear_ratio) / (2*PI)), 0.0f);
                (void)st;
            }
            break;
        }

        case ODRIVE_CMD_CLEAR_ERRORS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Clear_Errors(&odrives[i], tx);
                (void)st;
            }
            break;
        }

        case ODRIVE_CMD_SET_STATE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Axis_Requested_State(&odrives[i], tx, cmd->axis_state);
                (void)st;
            }
            break;
        }

        case ODRIVE_CMD_SET_CONTROLLER_MODE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Controller_Modes(&odrives[i], tx,
                         (Control_Mode)cmd->control_mode, (Input_Mode)cmd->input_mode);
                (void)st;
            }
            *current_ctrl_mode  = (Control_Mode)cmd->control_mode;
            *current_input_mode = (Input_Mode)cmd->input_mode;
            break;
        }

        case ODRIVE_CMD_STOP_ODRIVES:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Input_Vel(&odrives[i], tx, 0.0f, 0.0f);
                (void)st;
            }
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Axis_Requested_State(&odrives[i], tx, IDLE);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_CLEAR_ERRORS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Clear_Errors(&odrives[i], tx);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_SET_STATE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Axis_Requested_State(&odrives[i], tx, cmd->axis_state);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_SET_CTRL_MODE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Controller_Modes(&odrives[i], tx,
                         (Control_Mode)cmd->control_mode, (Input_Mode)cmd->input_mode);
                (void)st;
            }
            *current_ctrl_mode  = (Control_Mode)cmd->control_mode;
            *current_input_mode = (Input_Mode)cmd->input_mode;
            break;
        }

        case ODRIVE_CFG_SET_LIMITS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Limits(&odrives[i], tx, cmd->vel_limit, cmd->curr_limit);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_SET_POS_GAIN:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Position_Gain(&odrives[i], tx, cmd->pos_gain);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_SET_VEL_GAINS:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Vel_Gains(&odrives[i], tx, cmd->vel_gain, cmd->vel_int_gain);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_STARTUP:
        {
            HAL_StatusTypeDef startup_st = ODrive_Startup(
                odrives, num_odrives, tx,
                (Control_Mode)cmd->control_mode,
                (Input_Mode)cmd->input_mode,
                cmd->axis_state);
            if (startup_st == HAL_OK) {
                *current_ctrl_mode  = (Control_Mode)cmd->control_mode;
                *current_input_mode = (Input_Mode)cmd->input_mode;
            } else {
                FirmwareError_Push(FERR_STARTUP_FAILED, FERR_NO_AXIS, (uint8_t)startup_st);
            }
            break;
        }

        case ODRIVE_CFG_REBOOT:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Reboot_ODrive(&odrives[i], tx);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_SET_TORQUE:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Input_Torque(&odrives[i], tx, cmd->torque_ff[i]);
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_STOP:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                Set_Input_Vel(&odrives[i], tx, 0.0f, 0.0f);
            }
            osDelay(50);
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                Set_Axis_Requested_State(&odrives[i], tx, IDLE);
            }
            break;
        }

        case ODRIVE_CFG_SET_INPUT_POS:
        {
            if (*current_ctrl_mode != POSITION_CONTROL) break;
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Input_Pos(&odrives[i], tx,
                         cmd->input_pos_target,
                         (int16_t)(cmd->input_pos_vel_ff * 1000.0f),
                         (int16_t)(cmd->input_pos_trq_ff * 1000.0f));
                (void)st;
            }
            break;
        }

        case ODRIVE_CFG_SET_PARAM_FLOAT:
        {
            for (uint8_t i = 0; i < num_odrives; i++) {
                if (!(cmd->target_mask & (1 << i))) continue;
                FDCAN_WAIT_TX_FREE();
                st = Set_Param_Float(&odrives[i], tx,
                         cmd->param_endpoint_id, cmd->param_value);
                (void)st;
            }
            break;
        }

        default:
            break;
    }
}


void ODrive_UpdateTelemetryAndOdometry(Axis odrives[], uint8_t num_odrives,
                                       ODriveTelemetryMsg *msg, OdomData *odom,
                                       EKF *ekf,
                                       double x_offset, double y_offset, double radius,
                                       double u[4], double q_dot[3], uint32_t dt,
                                       double wheel_sign[])
{
    for (uint8_t i = 0; i < num_odrives; i++)
    {
        msg->node_id[i]            = odrives[i].NODE_ID;
        msg->axis_error[i]         = odrives[i].AXIS_Error;
        msg->axis_state[i]         = odrives[i].AXIS_Current_State;
        msg->controller_status[i]  = odrives[i].Controller_Status;
        msg->pos_est[i]            = odrives[i].AXIS_Encoder_Pos;
        msg->vel_est[i]            = odrives[i].AXIS_Encoder_Vel;
        msg->encoder_shadow[i]     = odrives[i].AXIS_Encoder_Shadow;
        msg->encoder_cpr[i]        = odrives[i].AXIS_Encoder_CPR;
        msg->bus_voltage[i]        = odrives[i].AXIS_Bus_Voltage;
        msg->bus_current[i]        = odrives[i].AXIS_Bus_Current;
        msg->iq_setpoint[i]        = odrives[i].AXIS_Iq_Setpoint;
        msg->iq_measured[i]        = odrives[i].AXIS_Iq_Measured;
        msg->updated[i]            = odrives[i].UPDATED;
        odrives[i].UPDATED         = 0;

        /* Convert velocity estimate back to rad/s in wheel frame (accounts for gear ratio).
         * Guard gear_ratio==0 (e.g. an axis whose NODE_ID never matched a heartbeat)
         * so we never emit inf/NaN into odometry, which would poison it permanently. */
        u[i] = (odrives[i].gear_ratio != 0)
             ? wheel_sign[i] * (odrives[i].AXIS_Encoder_Vel / odrives[i].gear_ratio) * 2.0 * PI
             : 0.0;
    }

    const double dt_s = dt * 0.001;

    /* 1. Predict — propagate state + covariance forward by dt. */
    ekf_predict(ekf, dt_s);

    /* 2. Correct on the wheel-derived body twist (mecanum FK, body frame).
     *    EKF_STUB=1 injects a synthetic constant velocity so the filter can
     *    be exercised on a bare board with no ODrives attached. */
#if EKF_STUB
    ekf_correct_wheel_twist(ekf, EKF_STUB_VX, EKF_STUB_VY, EKF_STUB_WZ);
#else
    double vx_body, vy_body, phi_dot_body;
    bodySpeedsFromUMecanum(x_offset, y_offset, radius, u,
                           &vx_body, &vy_body, &phi_dot_body);
    ekf_correct_wheel_twist(ekf, vx_body, vy_body, phi_dot_body);
#endif

    /* 3. Correct on the BNO085 (yaw + omega_z), gated by sequence # so each
     *    fresh IMU sample is consumed at most once. ekf_correct_imu() also
     *    honours ekf->use_imu and returns without doing anything when 0.
     *
     *    The BNO085 on this robot reports yaw + omega_z INVERTED vs the ROS
     *    base_link convention (rotating the robot +90 deg CCW makes raw yaw
     *    read -90 deg). This was previously papered over downstream by the
     *    home2 omnidriver dashboard, but with the EKF now on-MCU the sign
     *    has to be corrected BEFORE the filter integrates anything — else
     *    pose, twist, and the published quaternion all come out mirrored.
     *    Same physical cause for both: the IMU's z-axis is flipped vs
     *    base_link's z, so a single negation on each scalar is the right
     *    rotation. The raw `g_bno085_*` globals are left untouched so the
     *    `IMU_yaw` / `IMU_wz` UART fields stay backward-compatible with any
     *    existing consumer that has its own sign fix. */
    const uint32_t seq = g_bno085_seq;          /* atomic snapshot */
    const double   yaw_imu   = -(double)g_bno085_yaw;
    const double   omega_imu = -(double)g_bno085_wz;
    ekf_correct_imu(ekf, yaw_imu, omega_imu, seq);

    /* 4. Fill ROS-shaped OdomData from EKF state + covariance. */
    const double px = ekf->x[0], py = ekf->x[1], th = ekf->x[2];
    const double vxb = ekf->x[3], vyb = ekf->x[4], wz = ekf->x[5];
    const double c = cos(th), s = sin(th);

    /* Legacy fields (slim/fat printf compatibility). */
    odom->x_pos    = px;
    odom->y_pos    = py;
    odom->phi      = th;
    odom->q_dot[0] = wz;
    odom->q_dot[1] = c * vxb - s * vyb;   /* world x_dot */
    odom->q_dot[2] = s * vxb + c * vyb;   /* world y_dot */

    /* EKF-shaped fields. */
    odom->z_pos   = 0.0;
    const double half_th = 0.5 * th;
    odom->qx = 0.0f;
    odom->qy = 0.0f;
    odom->qz = (float)sin(half_th);
    odom->qw = (float)cos(half_th);
    odom->vx_body = vxb;
    odom->vy_body = vyb;
    odom->vz_body = 0.0;
    odom->wx = 0.0f;
    odom->wy = 0.0f;
    odom->wz = (float)wz;

    /* Map EKF 6x6 P into ROS-shaped covariance matrices.
     * Pose:  [ x,  y,  z,  roll, pitch, yaw ]
     *        idx 0=x, 1=y, 5=yaw — others marked unestimated with 1e9 on
     *        the diagonal, off-diagonals untouched (zero) per ROS convention.
     * Twist: [ vx, vy, vz, wx,   wy,    wz ]
     *        idx 0=vxb, 1=vyb, 5=omega — same convention. */
    const double LARGE_VAR = 1e9;
    for (int i = 0; i < 36; ++i) {
        odom->pose_covariance[i]  = 0.0;
        odom->twist_covariance[i] = 0.0;
    }
    /* Pose */
    odom->pose_covariance[0 * 6 + 0] = ekf->P[0 * EKF_N + 0];  /* x,x */
    odom->pose_covariance[0 * 6 + 1] = ekf->P[0 * EKF_N + 1];  /* x,y */
    odom->pose_covariance[0 * 6 + 5] = ekf->P[0 * EKF_N + 2];  /* x,yaw */
    odom->pose_covariance[1 * 6 + 0] = ekf->P[1 * EKF_N + 0];
    odom->pose_covariance[1 * 6 + 1] = ekf->P[1 * EKF_N + 1];
    odom->pose_covariance[1 * 6 + 5] = ekf->P[1 * EKF_N + 2];
    odom->pose_covariance[5 * 6 + 0] = ekf->P[2 * EKF_N + 0];
    odom->pose_covariance[5 * 6 + 1] = ekf->P[2 * EKF_N + 1];
    odom->pose_covariance[5 * 6 + 5] = ekf->P[2 * EKF_N + 2];
    odom->pose_covariance[2 * 6 + 2] = LARGE_VAR;  /* z  */
    odom->pose_covariance[3 * 6 + 3] = LARGE_VAR;  /* roll  */
    odom->pose_covariance[4 * 6 + 4] = LARGE_VAR;  /* pitch */
    /* Twist */
    odom->twist_covariance[0 * 6 + 0] = ekf->P[3 * EKF_N + 3];
    odom->twist_covariance[0 * 6 + 1] = ekf->P[3 * EKF_N + 4];
    odom->twist_covariance[0 * 6 + 5] = ekf->P[3 * EKF_N + 5];
    odom->twist_covariance[1 * 6 + 0] = ekf->P[4 * EKF_N + 3];
    odom->twist_covariance[1 * 6 + 1] = ekf->P[4 * EKF_N + 4];
    odom->twist_covariance[1 * 6 + 5] = ekf->P[4 * EKF_N + 5];
    odom->twist_covariance[5 * 6 + 0] = ekf->P[5 * EKF_N + 3];
    odom->twist_covariance[5 * 6 + 1] = ekf->P[5 * EKF_N + 4];
    odom->twist_covariance[5 * 6 + 5] = ekf->P[5 * EKF_N + 5];
    odom->twist_covariance[2 * 6 + 2] = LARGE_VAR;
    odom->twist_covariance[3 * 6 + 3] = LARGE_VAR;
    odom->twist_covariance[4 * 6 + 4] = LARGE_VAR;

    /* Keep the caller's q_dot[] in sync so existing consumers (none right
     * now, but the API still exposes it) see the same world twist we just
     * stored into odom->q_dot. */
    q_dot[0] = odom->q_dot[0];
    q_dot[1] = odom->q_dot[1];
    q_dot[2] = odom->q_dot[2];

    msg->timestamp_ms = osKernelGetTickCount();
}


osStatus_t ODrive_PushLatestTelemetry(osMessageQueueId_t queue, const ODriveTelemetryMsg *msg)
{
    osStatus_t st;
    ODriveTelemetryMsg discarded_msg;

    st = osMessageQueuePut(queue, msg, 0, 0);
    if (st == osOK) return osOK;

    if (st == osErrorResource)
    {
        if (osMessageQueueGet(queue, &discarded_msg, NULL, 0) == osOK)
            return osMessageQueuePut(queue, msg, 0, 0);
    }
    return st;
}


void UART_RX_ParseLine(const char *line_buf, ODriveCmdMsg *odrive_cmd,
                       osMessageQueueId_t UART_QueueHandle_arg,
                       osMessageQueueId_t URX_2_CAN_QueueHandle_arg)
{
    int msg_type = 0;
    osStatus_t qst;

    if (sscanf(line_buf, "%d", &msg_type) != 1) return;

    if (msg_type == 1) {
        double vx = 0.0, vy = 0.0, wz = 0.0;
        int parsed = sscanf(line_buf, "%*d %lf %lf %lf", &vx, &vy, &wz);
        if (parsed == 3) {
            odrive_cmd->type           = ODRIVE_CMD_SET_VEL;
            odrive_cmd->target_mask    = 0x0F;
            odrive_cmd->source         = CMD_SOURCE_ROS;
            odrive_cmd->buttons        = 0;
            odrive_cmd->robot_twist[0] = vx;
            odrive_cmd->robot_twist[1] = vy;
            odrive_cmd->robot_twist[2] = wz;
            for (int i = 0; i < 4; i++) odrive_cmd->torque_ff[i] = 0.0f;

            osMessageQueuePut(UART_QueueHandle_arg, odrive_cmd, 0, 0);
            osMessageQueuePut(URX_2_CAN_QueueHandle_arg, odrive_cmd, 0, 0);
        }

    } else if (msg_type == 2) {
        int sub_type = 0;
        unsigned int mask_u = 0x0F;

        if (sscanf(line_buf, "%*d %d %x", &sub_type, &mask_u) < 2) return;

        ODriveCmdMsg cfg_cmd = {0};
        cfg_cmd.type        = (uint8_t)sub_type;
        cfg_cmd.target_mask = (uint8_t)(mask_u & 0x0F);
        cfg_cmd.source      = CMD_SOURCE_ROS;

        switch ((ODriveCmdType)sub_type) {
            case ODRIVE_CFG_SET_STATE: {
                int state = 0;
                sscanf(line_buf, "%*d %*d %*x %d", &state);
                cfg_cmd.axis_state = (uint8_t)state;
                break;
            }
            case ODRIVE_CFG_SET_CTRL_MODE: {
                int cm = 0, im = 0;
                sscanf(line_buf, "%*d %*d %*x %d %d", &cm, &im);
                cfg_cmd.control_mode = (uint8_t)cm;
                cfg_cmd.input_mode   = (uint8_t)im;
                break;
            }
            case ODRIVE_CFG_SET_LIMITS: {
                float vl = 0.0f, cl = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f %f", &vl, &cl);
                cfg_cmd.vel_limit  = vl;
                cfg_cmd.curr_limit = cl;
                break;
            }
            case ODRIVE_CFG_SET_POS_GAIN: {
                float pg = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f", &pg);
                cfg_cmd.pos_gain = pg;
                break;
            }
            case ODRIVE_CFG_SET_VEL_GAINS: {
                float vg = 0.0f, vi = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f %f", &vg, &vi);
                cfg_cmd.vel_gain     = vg;
                cfg_cmd.vel_int_gain = vi;
                break;
            }
            case ODRIVE_CFG_STARTUP: {
                int cm = 2, im = 1, st = 8;
                sscanf(line_buf, "%*d %*d %*x %d %d %d", &cm, &im, &st);
                cfg_cmd.control_mode = (uint8_t)cm;
                cfg_cmd.input_mode   = (uint8_t)im;
                cfg_cmd.axis_state   = (uint8_t)st;
                break;
            }
            case ODRIVE_CFG_SET_TORQUE: {
                float tq = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f", &tq);
                cfg_cmd.torque_ff[0] = cfg_cmd.torque_ff[1] =
                cfg_cmd.torque_ff[2] = cfg_cmd.torque_ff[3] = tq;
                break;
            }
            case ODRIVE_CFG_SET_INPUT_POS: {
                float pos = 0.0f, vff = 0.0f, tff = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %f %f %f", &pos, &vff, &tff);
                cfg_cmd.input_pos_target = pos;
                cfg_cmd.input_pos_vel_ff = vff;
                cfg_cmd.input_pos_trq_ff = tff;
                break;
            }
            case ODRIVE_CFG_SET_PARAM_FLOAT: {
                unsigned int endpoint_id = 0;
                float value = 0.0f;
                sscanf(line_buf, "%*d %*d %*x %u %f", &endpoint_id, &value);
                cfg_cmd.param_endpoint_id = (uint16_t)endpoint_id;
                cfg_cmd.param_value       = value;
                break;
            }
            default:
                break;
        }

        osMessageQueuePut(URX_2_CAN_QueueHandle_arg, &cfg_cmd, 0, 0);

    }
}

/* USER CODE END Header_StartODriveTask */

void StartODriveTask(void *argument)
{
  /* USER CODE BEGIN StartODriveTask */

    const uint8_t num_odrives   = 4;
    const double  x_offset      = 0.195;
    const double  y_offset      = 0.195;
    const double  radius        = 0.0762;
    const double  wheel_sign[4] = { -1.0, 1.0, -1.0, 1.0 };

    odrives[0].NODE_ID     = 36;
    odrives[1].NODE_ID     = 34;
    odrives[2].NODE_ID     = 33;
    odrives[3].NODE_ID     = 40;
    odrives[0].gear_ratio  = 9;
    odrives[1].gear_ratio  = 9;
    odrives[2].gear_ratio  = 9;
    odrives[3].gear_ratio  = 9;

    ODriveSMState sm_state         = SM_BOOT;
    Control_Mode  current_ctrl_mode  = VELOCITY_CONTROL;
    Input_Mode    current_input_mode = PASSTHROUGH;

    /* BT source-priority override */
    const uint32_t BT_OVERRIDE_TIMEOUT_MS = 500;
    uint8_t  bt_override_active = 0;
    uint32_t last_bt_tick       = 0;

    /* General velocity-command watchdog.
     * If no ODRIVE_CMD_SET_VEL arrives from ANY source (ROS or BT) within
     * CMD_WATCHDOG_TIMEOUT_MS, we fire a single zero-velocity SET_VEL to all
     * four axes so the robot does not coast on its last command after the
     * controller has gone silent (e.g. ROS node crashed, USB unplugged,
     * BT link dropped). `cmd_watchdog_fired` is a latch so we don't spam
     * the CAN bus while the link is still down; it clears as soon as a new
     * SET_VEL arrives. */
    const uint32_t CMD_WATCHDOG_TIMEOUT_MS = 500;
    uint32_t last_vel_cmd_tick = osKernelGetTickCount();
    uint8_t  cmd_watchdog_fired = 0;
    /* While the command watchdog is latched, re-send the zero-velocity stop
     * periodically (a single stop frame can be lost) — rate-limited so we do
     * not flood the CAN bus from the 1 ms control loop. */
    uint32_t last_wd_stop_tick = 0;
    const uint32_t WD_STOP_RESEND_MS = 50;

    /* Periodic per-axis auto-recovery: re-arm any axis that has fallen out of
     * CLOSED_LOOP_CONTROL (undervoltage, fault, or its own CAN watchdog). */
    uint32_t last_rearm_tick = osKernelGetTickCount();
    const uint32_t REARM_PERIOD_MS = 500;

    /* Error-detection state — not involved in control, purely diagnostic. */
    uint32_t prev_axis_error[4]      = {0};        /* axis_error register transition detector */
    uint32_t last_hb_tick[4]         = {0};        /* last tick each axis sent a heartbeat */
    uint8_t  hb_timeout_reported[4]  = {0};        /* one-shot per timeout event */
    const uint32_t HB_TIMEOUT_MS     = 500;

    /* ESTOP release hold: the button must stay released for ESTOP_RELEASE_HOLD_MS
     * continuously before we re-arm. Any re-press during the hold cancels the timer. */
    uint8_t  estop_release_pending = 0;
    uint32_t estop_release_tick    = 0;
    uint32_t estop_clear_tick      = 0;

    const uint32_t boot_delay_ms = 3000;
    uint32_t boot_tick = osKernelGetTickCount();

    /* On-firmware 6-state EKF — replaces the previous (x, y, theta) Euler
     * dead-reckoning. EKF_TRUST_IMU toggles whether BNO085 corrections are
     * applied at run time; 0 = wheel-only kinematic EKF (functionally
     * dead-reckoning-equivalent, but with proper covariance accounting). */
    EKF ekf;
    ekf_init(&ekf, NULL);
    ekf.use_imu = EKF_TRUST_IMU;

    double u[4]     = {0.0};
    double q_dot[3] = {0.0};
    ODriveCmdMsg       cmd          = {0};
    ODriveTelemetryMsg telemetryMsg = {0};
    OdomData *odrive_odom = &telemetryMsg.odom;
    FDCAN_TXmsg tx = {0};

    uint32_t now, last_telem_tick = osKernelGetTickCount();
    /* Telemetry cadence drives BOTH the slim UART line and the EKF
     * predict+correct cycle — they share the same tick so the on-wire data
     * always matches a fresh EKF posterior. 20 ms ⇒ slim = 50 Hz, fat (every
     * 5th slim) = 10 Hz, EKF runs at 50 Hz which already matches the BNO085
     * report cadence so no IMU sample is ever skipped. */
    const uint32_t telemetry_period = 20;
    osStatus_t qst;

    /* BNO055 removed — orientation now provided by IMU_Task via g_bno085_* globals */
    osDelay(1);

    for (;;)
    {
        now = osKernelGetTickCount();

        /* ───── HARDWARE E-STOP (PE2 button) ─────────────────────────────
         * Sample-after-delay debounce: any edge on ESTOP_PIN sets
         * g_estop_sample_pending in the ISR. The task waits ESTOP_DEBOUNCE_MS
         * (250 ms) then reads the actual pin level. This is immune to contact
         * bounce on both press and release — no edge counting, no separate
         * press/release flags.  If another edge arrives before the window
         * expires the tick refreshes and the window restarts.
         *
         * Pin HIGH (NC opens = button pressed): enter SM_ESTOP, idle all axes.
         * Pin LOW  (NC closes = button released): re-arm all axes, SM_RUNNING. */
        if (g_estop_sample_pending &&
            (now - g_estop_sample_tick) >= ESTOP_DEBOUNCE_MS)
        {
            g_estop_sample_pending = 0;

            if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_SET) {
                /* Button is pressed — engage ESTOP immediately, cancel any
                 * pending release timer. */
                estop_release_pending = 0;
                if (sm_state != SM_ESTOP) {
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        FDCAN_WAIT_TX_FREE();
                        Set_Input_Vel(&odrives[i], &tx, 0.0f, 0.0f);
                    }
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        FDCAN_WAIT_TX_FREE();
                        Set_Axis_Requested_State(&odrives[i], &tx, IDLE);
                    }
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        FDCAN_WAIT_TX_FREE();
                        Clear_Errors(&odrives[i], &tx);
                    }
                    sm_state           = SM_ESTOP;
                    g_estop_active     = 1;
                    cmd_watchdog_fired = 0;
                    last_vel_cmd_tick  = now;
                    HAL_GPIO_WritePin(ESTOP_LED_PORT, ESTOP_LED_PIN, GPIO_PIN_SET);
                }
            } else {
                /* Button is released — start the hold timer. Re-arm only after
                 * ESTOP_RELEASE_HOLD_MS of continuous release (checked below). */
                if (sm_state == SM_ESTOP) {
                    estop_release_pending = 1;
                    estop_release_tick    = now;
                }
            }
        }

        /* ESTOP release hold: re-arm once the button has been released
         * continuously for ESTOP_RELEASE_HOLD_MS. Final pin read confirms
         * release is stable. Any re-press cancels estop_release_pending above. */
        if (estop_release_pending &&
            sm_state == SM_ESTOP &&
            (now - estop_release_tick) >= ESTOP_RELEASE_HOLD_MS)
        {
            estop_release_pending = 0;
            if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET) {
                for (uint8_t i = 0; i < num_odrives; i++) {
                    FDCAN_WAIT_TX_FREE();
                    Set_Input_Pos(&odrives[i], &tx,
                                  odrives[i].AXIS_Encoder_Pos, 0, 0);
                }
                for (uint8_t i = 0; i < num_odrives; i++) {
                    ODrive_ArmAxisConfirmed(&odrives[i], &tx,
                                           current_ctrl_mode,
                                           current_input_mode, 5, i);
                }
                sm_state          = SM_RUNNING;
                g_estop_active    = 0;
                last_vel_cmd_tick = now;
                HAL_GPIO_WritePin(ESTOP_LED_PORT, ESTOP_LED_PIN, GPIO_PIN_RESET);
            }
        }

        /* While in SM_ESTOP the rest of the loop is still allowed to run,
         * but every branch that could un-IDLE the axes is gated:
         *   - CAN bus-off recovery: already guarded on sm_state == SM_RUNNING.
         *   - SM_RUNNING auto-rearm block: lives inside case SM_RUNNING, so
         *     the switch's case SM_ESTOP (no-op) skips it.
         *   - Command queue gets consumed but the switch ignores it.
         * The telemetry path at the bottom still runs, so the dashboard sees
         * live EKF + axis data during the e-stop — useful for diagnosing
         * "what happened?" and for visualising the operator pushing the base. */
        /* ───────────────────────────────────────────────────────────────── */

        /* CAN bus-off recovery FIRST: STM32H7 FDCAN does not self-recover, and
         * while bus-off NOTHING can be transmitted. Recover the peripheral, then
         * STAY in SM_RUNNING and let the per-axis re-arm (below) bring any axis
         * the ODrive watchdog disarmed back to CLOSED_LOOP automatically — do NOT
         * force SM_IDLE (that would need a manual "start all"). The ODrive on-board
         * CAN watchdog is the runaway backstop while the bus is down. We send one
         * zero-velocity so a stale setpoint can't resume the instant the bus is back. */
        if (FDCAN_RecoverIfBusOff() && sm_state == SM_RUNNING) {
            ODriveCmdMsg zero_cmd = {0};
            zero_cmd.type        = ODRIVE_CMD_SET_VEL;
            zero_cmd.target_mask = 0x0F;
            ODrive_ProcessCommand(&zero_cmd, odrives, num_odrives, &tx,
                odrive_odom, x_offset, y_offset, radius,
                &current_ctrl_mode, &current_input_mode,
                (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
        }

        /* Read BNO085 data from IMU_Task shared globals.
         * Each float read is atomic on Cortex-M7 (32-bit aligned load).
         * Euler angles are converted radians → degrees to preserve backward
         * compatibility with the previous BNO055 output format (degrees).
         * Quaternion, angular velocity, and linear acceleration are copied
         * raw (rad, rad/s, m/s^2) for the ROS sensor_msgs/Imu message. */
        telemetryMsg.imu.yaw   = (double)g_bno085_yaw   * RAD_TO_DEG_D;
        telemetryMsg.imu.roll  = (double)g_bno085_roll  * RAD_TO_DEG_D;
        telemetryMsg.imu.pitch = (double)g_bno085_pitch * RAD_TO_DEG_D;

        telemetryMsg.imu.qx = g_bno085_qx;
        telemetryMsg.imu.qy = g_bno085_qy;
        telemetryMsg.imu.qz = g_bno085_qz;
        telemetryMsg.imu.qw = g_bno085_qw;

        telemetryMsg.imu.wx = g_bno085_wx;
        telemetryMsg.imu.wy = g_bno085_wy;
        telemetryMsg.imu.wz = g_bno085_wz;

        telemetryMsg.imu.ax = g_bno085_ax;
        telemetryMsg.imu.ay = g_bno085_ay;
        telemetryMsg.imu.az = g_bno085_az;

        qst = osMessageQueueGet(URX_2_CAN_QueueHandle, &cmd, NULL, 2);

        /* Keep the velocity-command watchdog disarmed and timestamp fresh
         * whenever we are not actively driving. This prevents an immediate
         * fire on the next SM_BOOT/SM_STARTUP/SM_IDLE → SM_RUNNING transition
         * after a long pause. */
        if (sm_state != SM_RUNNING) {
            last_vel_cmd_tick  = now;
            cmd_watchdog_fired = 0;
        }

        /* If we just received a velocity command from any source, refresh the
         * watchdog. We do this before the state machine so a SET_VEL that is
         * later dropped (e.g. ROS command masked by an active BT override)
         * still proves the upstream link is alive. */
        if (qst == osOK && cmd.type == ODRIVE_CMD_SET_VEL) {
            last_vel_cmd_tick  = now;
            cmd_watchdog_fired = 0;
        }

        switch (sm_state)
        {
            case SM_BOOT:
            {
                if (qst == osOK && cmd.type == ODRIVE_CFG_STARTUP) {
                    sm_state = SM_STARTUP;
                    ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                    sm_state = SM_RUNNING;
                }
                else if ((now - boot_tick) >= boot_delay_ms) {
                    sm_state = SM_STARTUP;

                    /* Wait for the first heartbeat from every axis: this proves
                     * the CAN bus is up AND each S1 finished its own power-on
                     * before we command CLOSED_LOOP. A fixed delay races the S1
                     * boot time and silently loses the arm command. AXIS_Current_State
                     * stays UNDEFINED(0) until a heartbeat arrives; bounded so a
                     * truly absent S1 cannot hang us forever.
                     * CAN_STUB=1: loopback never produces heartbeats — skip. */
#if !CAN_STUB
                    for (uint16_t hb = 0; hb < 300; hb++) {      /* up to ~3 s */
                        uint8_t all_seen = 1;
                        for (uint8_t i = 0; i < num_odrives; i++)
                            if (odrives[i].AXIS_Current_State == UNDEFINED) all_seen = 0;
                        if (all_seen) break;
                        /* Drain (not just ignore) any BT/ROS command that
                         * arrives during the wait. URX_2_CAN_QueueHandle is
                         * depth 3 and nothing else reads it while we're stuck
                         * in this loop -- without this, an active BT stream
                         * fills it almost instantly and floods FERR_BT_QUEUE_FULL
                         * every boot. Commands during arming are expected to
                         * be dropped, so this is a silent discard, not an error. */
                        {
                            ODriveCmdMsg discard;
                            while (osMessageQueueGet(URX_2_CAN_QueueHandle, &discard, NULL, 0) == osOK) {}
                        }
                        osDelay(10);
                    }
#endif

                    /* Push the project-tuned velocity-loop gains before arming.
                     * The auto-arm path uses ODrive_ArmAxisConfirmed which only
                     * touches Clear_Errors / Set_Controller_Modes / Set_Axis_
                     * Requested_State — it does NOT carry gains. Setting them
                     * here (after heartbeats prove the bus + each S1 are alive)
                     * means the first SET_INPUT_VEL frame sees the tuned gains. */
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        FDCAN_WAIT_TX_FREE();
                        Set_Vel_Gains(&odrives[i], &tx,
                                     ODRIVE_STARTUP_VEL_GAIN,
                                     ODRIVE_STARTUP_VEL_INT_GAIN);
                    }

                    /* Push the tuned vel_ramp_rate (RxSdo arbitrary-parameter
                     * write) before arming, same RAM-only/every-boot pattern
                     * as the gains above. */
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        FDCAN_WAIT_TX_FREE();
                        Set_Param_Float(&odrives[i], &tx,
                                       ODRIVE_VEL_RAMP_RATE_ENDPOINT_ID,
                                       ODRIVE_STARTUP_VEL_RAMP_RATE);
                    }

                    /* Arm each axis WITH heartbeat confirmation + retry, instead
                     * of assuming a queued CAN frame took effect. */
                    for (uint8_t i = 0; i < num_odrives; i++)
                        ODrive_ArmAxisConfirmed(&odrives[i], &tx,
                                               VELOCITY_CONTROL, VEL_RAMP, 5, i);
                    current_ctrl_mode  = VELOCITY_CONTROL;
                    current_input_mode = VEL_RAMP;
                    sm_state = SM_RUNNING;
                }
                break;
            }

            case SM_STARTUP:
                sm_state = SM_RUNNING;
                break;

            case SM_RUNNING:
            {
                /* Periodic per-axis auto-recovery: if an axis fell out of
                 * CLOSED_LOOP_CONTROL (undervoltage, latched fault, or its own
                 * CAN watchdog tripping), clear errors, re-arm it, and zero its
                 * setpoint so it does not lurch on recovery. Rate-limited so it
                 * never starves the SET_VEL stream. This also fixes "the S1's
                 * drop out over time and never come back" without a reset. */
                if ((now - last_rearm_tick) >= REARM_PERIOD_MS) {
                    last_rearm_tick = now;
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        if (odrives[i].AXIS_Current_State != UNDEFINED &&
                            odrives[i].AXIS_Current_State != CLOSED_LOOP_CONTROL) {
                            FirmwareError_Push(FERR_AXIS_REARM, i,
                                               (uint8_t)odrives[i].AXIS_Current_State);
                            FDCAN_WAIT_TX_FREE(); Clear_Errors(&odrives[i], &tx);
                            FDCAN_WAIT_TX_FREE(); Set_Axis_Requested_State(&odrives[i], &tx, CLOSED_LOOP_CONTROL);
                            FDCAN_WAIT_TX_FREE(); Set_Input_Vel(&odrives[i], &tx, 0.0f, 0.0f);
                        }
                    }
                }

                /* General command watchdog: if no SET_VEL has arrived from
                 * any source for CMD_WATCHDOG_TIMEOUT_MS, send a single
                 * zero-velocity SET_VEL to all four axes and latch so we
                 * don't repeatedly retransmit on a dead link. The latch
                 * clears as soon as a new SET_VEL arrives (above). */
                if ((now - last_vel_cmd_tick) >= CMD_WATCHDOG_TIMEOUT_MS) {
                    if (!cmd_watchdog_fired) {
                        cmd_watchdog_fired = 1;
                        FirmwareError_Push(FERR_CMD_WATCHDOG, FERR_NO_AXIS, 0);
                        /* Release BT override so ROS regains control as soon as
                         * packets resume. */
                        bt_override_active = 0;
                    }
                    /* Re-send zero velocity periodically (NOT one-shot): a single
                     * stop frame can be lost and the link may still be dead. The
                     * motors resume automatically when a fresh SET_VEL arrives
                     * (which clears the latch, above). */
                    if ((now - last_wd_stop_tick) >= WD_STOP_RESEND_MS) {
                        last_wd_stop_tick = now;
                        ODriveCmdMsg zero_cmd = {0};
                        zero_cmd.type        = ODRIVE_CMD_SET_VEL;
                        zero_cmd.target_mask = 0x0F;
                        ODrive_ProcessCommand(&zero_cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                    }
                }

                /* BT watchdog: if no BT packet for BT_OVERRIDE_TIMEOUT_MS, stop and release */
                if (bt_override_active && (now - last_bt_tick) >= BT_OVERRIDE_TIMEOUT_MS) {
                    ODriveCmdMsg zero_cmd = {0};
                    zero_cmd.type        = ODRIVE_CMD_SET_VEL;
                    zero_cmd.target_mask = 0x0F;
                    ODrive_ProcessCommand(&zero_cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                    bt_override_active     = 0;
                    telemetryMsg.bt_active = BT_active;
                }

                if (qst == osOK) {
                    /* Bit 3 of buttons: emergency stop — bypasses source priority */
                    if (cmd.buttons & BT_ESTOP_BUTTON) {
                        ODriveCmdMsg stop_cmd = {0};
                        stop_cmd.type        = ODRIVE_CMD_STOP_ODRIVES;
                        stop_cmd.target_mask = 0x0F;
                        ODrive_ProcessCommand(&stop_cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                        sm_state = SM_IDLE;
                        break;
                    }

                    /* STOP is a safety command: honor it UNCONDITIONALLY, before
                     * any source-priority / BT-override masking. Otherwise a web
                     * "stop" from ROS is silently dropped while a BT controller
                     * holds the override (the base keeps driving). */
                    if (cmd.type == ODRIVE_CFG_STOP || cmd.type == ODRIVE_CMD_STOP_ODRIVES) {
                        ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                        bt_override_active = 0;
                        sm_state = SM_IDLE;
                        break;
                    }

                    if (cmd.source == CMD_SOURCE_BT) {
                        if (BT_active == 2) {
                            /* Active: BT is controlling the robot */
                            bt_override_active = 1;
                            last_bt_tick       = now;
                        } else {
                            /* Paired or Inactive: toggle-off stop packet — release override */
                            bt_override_active = 0;
                        }
                        telemetryMsg.bt_active = BT_active;
                        telemetryMsg.bt_vx = (float)cmd.robot_twist[0];
                        telemetryMsg.bt_vy = (float)cmd.robot_twist[1];
                        telemetryMsg.bt_wz = (float)cmd.robot_twist[2];
                    } else {
                        /* ROS source: silently drop while BT override is active */
                        if (bt_override_active && (now - last_bt_tick) < BT_OVERRIDE_TIMEOUT_MS)
                            break;
                    }

                    if (cmd.type == ODRIVE_CFG_STOP) {
                        ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                        sm_state = SM_IDLE;
                        break;
                    }
                    if (cmd.type == ODRIVE_CFG_REBOOT) {
                        ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                            odrive_odom, x_offset, y_offset, radius,
                            &current_ctrl_mode, &current_input_mode,
                            (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                        sm_state  = SM_BOOT;
                        boot_tick = osKernelGetTickCount();
                        break;
                    }
                    ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                }
                break;
            }

            case SM_IDLE:
            {
                if (qst == osOK && cmd.type == ODRIVE_CFG_STARTUP) {
                    ODrive_ProcessCommand(&cmd, odrives, num_odrives, &tx,
                        odrive_odom, x_offset, y_offset, radius,
                        &current_ctrl_mode, &current_input_mode,
                        (double*)wheel_sign, telemetryMsg.IK_computed_wheel_speeds);
                    sm_state = SM_RUNNING;
                }
                break;
            }
            case SM_ESTOP:
                /* Periodically clear ODrive errors so the motors stay in
                 * a freely-backdrivable IDLE state even if the ODrive latches
                 * a new fault while the button is held (e.g. encoder error
                 * from someone pushing the base). */
                if ((now - estop_clear_tick) >= 500u) {
                    estop_clear_tick = now;
                    for (uint8_t i = 0; i < num_odrives; i++) {
                        FDCAN_WAIT_TX_FREE();
                        Clear_Errors(&odrives[i], &tx);
                    }
                }
                break;
        }

        uint32_t delta_t = now - last_telem_tick;
        if (delta_t >= telemetry_period) {
            ODrive_UpdateTelemetryAndOdometry(
                odrives, num_odrives, &telemetryMsg, odrive_odom,
                &ekf,
                x_offset, y_offset, radius,
                u, q_dot, delta_t, (double*)wheel_sign);

            /* Per-axis error detection (diagnostic only, no control impact). */
            for (uint8_t i = 0; i < num_odrives; i++) {
                /* Axis fault: AXIS_Error register went non-zero. */
                if (telemetryMsg.axis_error[i] != 0 && prev_axis_error[i] == 0)
                    FirmwareError_Push(FERR_AXIS_FAULT, i,
                                       (uint8_t)(telemetryMsg.axis_error[i] & 0xFF));
                prev_axis_error[i] = telemetryMsg.axis_error[i];

                /* Heartbeat timeout: axis stopped sending CAN heartbeats. */
                if (telemetryMsg.updated[i]) {
                    last_hb_tick[i] = now;
                    hb_timeout_reported[i] = 0;
                } else if (sm_state == SM_RUNNING &&
                           last_hb_tick[i] != 0 &&
                           (now - last_hb_tick[i]) > HB_TIMEOUT_MS &&
                           !hb_timeout_reported[i]) {
                    FirmwareError_Push(FERR_HEARTBEAT_TIMEOUT, i, 0);
                    hb_timeout_reported[i] = 1;
                }
            }

            /* Snapshot the firmware SM state on every push — let the host
             * distinguish SM_IDLE (commanded) from SM_ESTOP (button held). */
            telemetryMsg.sm_state = (uint8_t)sm_state;

            ODrive_PushLatestTelemetry(CAN_2_UTX_QueueHandle, &telemetryMsg);
            last_telem_tick = now;
        }

        osDelay(1);
    }
  /* USER CODE END StartODriveTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) { HAL_IncTick(); }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
