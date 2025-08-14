/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "mcp2515.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/** @brief I2C bus handle, used for BNO055 readings */
I2C_HandleTypeDef hi2c1;

/** @brief SPI port handle, used for MCP2515 CAN implementation */
SPI_HandleTypeDef hspi1;

/** @brief Timer 1 handle, CH1 (PE9) & CH2 (PE11) used in encoder mode for Encoder 1 */
TIM_HandleTypeDef htim1;

/** @brief Timer 2 handle, CH1 (PA15) & CH2 (PB3) used in encoder mode for Encoder 2 */
TIM_HandleTypeDef htim2;

/** @brief Timer 4 handle, CH1 (PD12) & CH2 (PD13) used in encoder mode for Encoder 3 */
TIM_HandleTypeDef htim4;

/** @brief Timer 5 handle, CH1 (PA0) used in PWM mode for H-Bridge_1 ENA */
TIM_HandleTypeDef htim5;

/** @brief Timer 8 handle, CH1 (PC6) & CH2 (PC7) used in encoder mode for Encoder 4 */
TIM_HandleTypeDef htim8;

/** @brief Timer 12 handle, CH1 (PE5) used in PWM mode for H-Bridge_1 ENB */
TIM_HandleTypeDef htim12;

/** @brief Timer 13 handle, currently unused */
TIM_HandleTypeDef htim13;

/** @brief Timer 14 handle, CH1 (PB14) used in PWM mode for H-Bridge_2 ENA */
TIM_HandleTypeDef htim14;

/** @brief Timer 15 handle, CH1 (PF9) used in PWM mode for H-Bridge_2 ENB */
TIM_HandleTypeDef htim15;

/** @brief uart3 handle, connected to microusb port by default, used for serial communication with computer with ros2 */
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UART_TX_Task */
osThreadId_t UART_TX_TaskHandle;
const osThreadAttr_t UART_TX_Task_attributes = {
  .name = "UART_TX_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UART_Queue */
osMessageQueueId_t UART_QueueHandle;
const osMessageQueueAttr_t UART_Queue_attributes = {
  .name = "UART_Queue"
};
/* Definitions for MutexUART_Data */
osMutexId_t MutexUART_DataHandle;
const osMutexAttr_t MutexUART_Data_attributes = {
  .name = "MutexUART_Data"
};
/* USER CODE BEGIN PV */
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
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
void StartDefaultTask(void *argument);
void start_UART_RX_Task(void *argument);
void Start_UART_TX_Task(void *argument);
void StartControlTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Computes the delta between two encoder counter values.
 * @details
 * Subtracts the previous encoder count from the current count and casts
 * the result to a signed 16-bit integer to handle the wrap around
 * example with 16bit encoder counters: 10 - 65530 = (int16) -16.
 *
 * @param[in] current Current encoder count.
 * @param[in] previous Previous encoder count.
 * @return Signed difference in counts.
 */
int16_t computeDeltaCNT(uint16_t current, uint16_t previous) {
	int16_t delta = (int16_t)(current - previous);
	return delta;
}


/**
* @brief Sets the direction of a motor using two GPIO pins.
* @details
* Drives the motor direction pins based on the given `dir` value:
* - `dir == 0`: Sets pin1 low, pin2 high.
* - `dir == 1`: Sets pin1 high, pin2 low.
* - `dir >= 2`: Sets both pins low (motor off/brake).
*
* Modifies the port's Output Data Register (`ODR`) directly.
*
* @param[in] port Pointer to the GPIO port structure.
* @param[in] pin1 First GPIO pin number (0–15).
* @param[in] pin2 Second GPIO pin number (0–15).
* @param[in] dir Direction code (0=reverse, 1=forward, 2=off).
*/
void setMotorDirection(GPIO_TypeDef *port, uint16_t pin1, uint16_t pin2, uint8_t dir) {
	if (dir < 2) {
		dir ? (port->ODR |= (1 << pin1)) : (port->ODR &= ~(1 << pin1));
		!dir ? (port->ODR |= (1 << pin2)) : (port->ODR &= ~(1 << pin2));
	} else {
		port->ODR &= ~(1 << pin1);
		port->ODR &= ~(1 << pin2);
	}
}

/**
* @brief Computes wheel angular velocities for a 4-wheel omni robot.
* @details
* Uses robot orientation `phi`, platform radius `d`, wheel radius `r`,
* and desired body-frame velocities (`phi_dot`, `y_dot`, `x_dot`) to
* calculate per-wheel angular speeds `u[0..3]`.
*
* @param[in] phi Robot heading in radians.
* @param[in] d Distance from robot center to wheel axis.
* @param[in] r Wheel radius.
* @param[out] u Array of 4 wheel angular velocities (rad/s).
* @param[in] phi_dot Desired angular velocity about Z axis (rad/s).
* @param[in] y_dot Desired Y velocity in body frame (m/s).
* @param[in] x_dot Desired X velocity in body frame (m/s).
* @return Always returns 0.
*/
int computeNecessaryWheelSpeedsOmni(double phi, double d, double r, double u[4], double phi_dot, double y_dot, double x_dot) {
	u[0] = (d * phi_dot + y_dot * cos(phi) - x_dot * sin(phi)) / r;
    u[1] = (d * phi_dot - x_dot * cos(phi) - y_dot * sin(phi)) / r;
    u[2] = (d * phi_dot - y_dot * cos(phi) + x_dot * sin(phi)) / r;
    u[3] = (d * phi_dot + x_dot * cos(phi) + y_dot * sin(phi)) / r;

    return 0;
}


/**
* @brief Computes wheel angular velocities for a mecanum-wheel robot.
* @details
* Uses robot orientation `phi`, wheel offsets (`x_off`, `y_off`), wheel
* radius `r`, and desired body-frame velocities to compute per-wheel
* angular speeds `u[0..3]`.
*
* @param[in] phi Robot heading in radians.
* @param[in] x_off Wheel X offset from robot center.
* @param[in] y_off Wheel Y offset from robot center.
* @param[in] r Wheel radius.
* @param[out] u Array of 4 wheel angular velocities (rad/s).
* @param[in] phi_dot Desired angular velocity about Z axis (rad/s).
* @param[in] y_dot Desired Y velocity in body frame (m/s).
* @param[in] x_dot Desired X velocity in body frame (m/s).
* @return Always returns 0.
*/
int computeNecessaryWheelSpeedsMecanum(double phi, double x_off, double y_off, double r, double u[4], double phi_dot, double y_dot, double x_dot) {
	u[0] = (x_dot*(cos(phi) + sin(phi)) - y_dot*(cos(phi) - sin(phi)) - phi_dot*(x_off + y_off)) / r;
    u[1] = (x_dot*(cos(phi) - sin(phi)) + y_dot*(cos(phi) + sin(phi)) + phi_dot*(x_off + y_off)) / r;
    u[2] = (x_dot*(cos(phi) + sin(phi)) - y_dot*(cos(phi) - sin(phi)) + phi_dot*(x_off + y_off)) / r;
    u[3] = (x_dot*(cos(phi) - sin(phi)) + y_dot*(cos(phi) + sin(phi)) - phi_dot*(x_off + y_off)) / r;

    return 0;
}

/**
* @brief Computes global angular and linear velocities from omni-wheel speeds.
* @details
* Converts per-wheel angular velocities `u[0..3]` into global frame
* velocities `q_dot[0]` (angular), `q_dot[1]` (X), and `q_dot[2]` (Y)
* using robot heading `phi`, distance to wheel `d`, and wheel radius `r`.
*
* @param[in] phi Robot heading in radians.
* @param[in] d Distance from robot center to wheel axis.
* @param[in] r Wheel radius.
* @param[in] u Array of 4 wheel angular velocities (rad/s).
* @param[out] q_dot Array of 3 global velocities: {phi_dot, x_dot, y_dot}.
* @return Always returns 0.
*/
int globalSpeedsFromUOmni(double phi, double d, double r, double u[4], double q_dot[3]) {
    q_dot[0] = (u[0] + u[1] + u[2] + u[3]) / (4 * d); // Angular velocity
    q_dot[1] = -cos(phi)*((u[1] - u[3])/2) - sin(phi)*((u[0] - u[2])/2); // X velocity
    q_dot[2] = cos(phi)*((u[0] - u[2])/2) - sin(phi)*((u[1] - u[3])/2); // Y velocity

    return 0;
}

/**
* @brief Computes global angular and linear velocities from mecanum-wheel speeds.
* @details
* Converts per-wheel angular velocities `u[0..3]` into global frame
* velocities `q_dot[0]` (angular), `q_dot[1]` (X), and `q_dot[2]` (Y)
* using robot heading `phi`, wheel offsets (`x_off`, `y_off`), and
* wheel radius `r`.
*
* @param[in] phi Robot heading in radians.
* @param[in] x_off Wheel X offset from robot center.
* @param[in] y_off Wheel Y offset from robot center.
* @param[in] r Wheel radius.
* @param[in] u Array of 4 wheel angular velocities (rad/s).
* @param[out] q_dot Array of 3 global velocities: {phi_dot, x_dot, y_dot}.
* @return Always returns 0.
*/
int globalSpeedsFromUMecanum(double phi, double x_off, double y_off, double r, double u[4], double q_dot[3]) {
    q_dot[0] = (u[1] - u[0] + u[2] - u[3]) / ((4 * (x_off + y_off)) / r); // Angular velocity
    q_dot[1] = cos(phi)*(r/4)*(u[0] + u[1] + u[2] + u[3]) + sin(phi)*(r/4)*(u[0] - u[1] + u[2] - u[3]); // X velocity
    q_dot[2] = sin(phi)*(r/4)*(u[0] + u[1] + u[2] + u[3]) - cos(phi)*(r/4)*(u[0] - u[1] + u[2] - u[3]); // X velocity

    return 0;
}

//double checkFloatRx( uint8_t floatRx[], uint8_t floatRx_size){
//	double receivedFloat = 0;
//	float pi = 3.141592;
//
//	if (floatRx[0] == 'p' && floatRx[1] == 'i'){
//		receivedFloat = pi;
//	}
//	for (int i = 0; i < floatRx_size; i++) {
//
//	}
//}

//const float tau = 0.05f;
//float alpha = dt / (tau + dt);
//filteredSpeed = alpha * measuredSpeed + (1.0f - alpha) * filteredSpeed;


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
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  setvbuf(stdin, NULL, _IONBF, 0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
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
  MX_I2C1_Init();
  MX_SPI1_Init();
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
//  } else {
//	  printf("TODO BN\r\n");
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of MutexUART_Data */
  MutexUART_DataHandle = osMutexNew(&MutexUART_Data_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART_Queue */
  UART_QueueHandle = osMessageQueueNew (10, sizeof(double), &UART_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  UART_QueueHandle = osMessageQueueNew (3, sizeof(InputData), &UART_Queue_attributes);
  UART2CtrlTsk_QueueHandle = osMessageQueueNew (3, sizeof(InputData), &UART2CtrlTsk_Queue_attributes);
  CtrlTsk_QueueHandle = osMessageQueueNew (3, sizeof(CtrlTsk_Data), &CtrlTsk_Queue_attributes);
  UART2KPIDs_QueueHandle = osMessageQueueNew (3, sizeof(PIDConfig), &UART2KPIDs_Queue_attributes);
  kpids_UART_TX_QueueHandle = osMessageQueueNew (3, sizeof(PIDConfig), &kpids_UART_TX_Queue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_RX_Task */
//  UART_RX_TaskHandle = osThreadNew(start_UART_RX_Task, NULL, &UART_RX_Task_attributes);

  /* creation of UART_TX_Task */
//  UART_TX_TaskHandle = osThreadNew(Start_UART_TX_Task, NULL, &UART_TX_Task_attributes);

  /* creation of ControlTask */
//  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 239;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 239;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 239;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 239;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 19999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 239;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
   GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOD, PD4_Pin|PD5_Pin|PD6_Pin|PD7_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOE, PE2_Pin|PE4_Pin|PE3_Pin|PE6_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOG, MCP2515_CS_Pin, GPIO_PIN_RESET);

   // Declare IN1, IN2, IN3 IN4 of H Bridge, in that order
   GPIO_InitStruct.Pin = PD4_Pin|PD5_Pin|PD6_Pin|PD7_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


   GPIO_InitStruct.Pin = MCP2515_CS_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(MCP2515_CS_GPIO_Port, &GPIO_InitStruct);

   // Declare IN1, IN2, IN3 IN4 of H Bridge, in that order
   GPIO_InitStruct.Pin = PE2_Pin|PE4_Pin|PE3_Pin|PE6_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

//   GPIOD->ODR ^= (0x1UL << 4U);
//   GPIOD->ODR ^= (0x1UL << 5U);
//   GPIOD->ODR ^= (0x1UL << 6U);
//   GPIOD->ODR ^= (0x1UL << 7U);
//
//   GPIOE->ODR ^= (0x1UL << 2U);
//   GPIOE->ODR ^= (0x1UL << 3U);
//   GPIOE->ODR ^= (0x1UL << 4U);
//   GPIOE->ODR ^= (0x1UL << 6U);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    MX_SPI1_Init();

    static struct can_frame tx_frame;
    static struct can_frame rx_frame;
    CAN_Error err;

    err = MCP_reset();
    if (err != ERROR_OK) {
        printf("MCP_reset failed! Error code: %d\n\r", err);
        return;  // abort or handle error
    }

    // 2. Bitrate (optional: can remove if using setBitrateClock only)
    err = MCP_setBitrate(CAN_500KBPS);
    if (err != ERROR_OK) {
        printf("MCP_setBitrate failed! Error code: %d\n\r", err);
        return;
    }

    // 3. Bitrate + Clock
    err = MCP_setBitrateClock(CAN_500KBPS, MCP_8MHZ);
    if (err != ERROR_OK) {
        printf("MCP_setBitrateClock failed! Error code: %d\n\r", err);
        return;
    }

    osDelay(10);

    err = MCP_setNormalMode();
    if (err != ERROR_OK) {
        printf("Failed to enter Normal mode! Error code: %d\n\r", err);
    } else {
        printf("MCP2515 is in Normal mode\n\r");
    }

    uint8_t canstat = MCP_readRegister(0x0E);  // 0x0E
    uint8_t mode = canstat & 0xE0;  // Mask bits 7:5
    printf("CANSTAT = 0x%02X\n\r", canstat);

    memset(&tx_frame, 0, sizeof(tx_frame));
//    tx_frame.can_id = 0x799;
//    tx_frame.can_dlc = 8;
//    memcpy(tx_frame.data, "STM32H7-", 8);

//    tx_frame.can_id  = 0x29 | CAN_RTR_FLAG;  // 0x29 = (1 << 5) | 0x09
//    tx_frame.can_dlc = 0;

    uint8_t node_id = 33;
    uint32_t state = 8;  // AXIS_STATE_CLOSED_LOOP_CONTROL

    tx_frame.can_id = (node_id << 5) | 0x007;  // 0x007 = Set_Axis_State
    tx_frame.can_dlc = 4;

    memcpy(&tx_frame.data[0], &state, sizeof(state));

    MCP_sendMessage(&tx_frame);

    osDelay(300);

    tx_frame.can_dlc = 8;
    float velocity = 1.4f;
    float torque_ff = 0.0f;  // Optional
    int16_t torque_ff_pos = 0;           // torque feedforward = 0
    int16_t vel_ff = 0;              // velocity feedforward = 0
    float position = 0.0f;          // revolutions


    tx_frame.can_id = (node_id << 5) | 0x00D;  // 0x00D = Set_Input_Vel
    memcpy(&tx_frame.data[0], &velocity, sizeof(float));
    memcpy(&tx_frame.data[4], &torque_ff, sizeof(float));

//    tx_frame.can_id = (node_id << 5) | 0x00C;  // 0x00C = Set_Input_Pos
//    memcpy(&tx_frame.data[0], &position, sizeof(float));
//    memcpy(&tx_frame.data[4], &vel_ff, sizeof(int16_t));
//    memcpy(&tx_frame.data[6], &torque_ff_pos, sizeof(int16_t));

//    for (int i = 0; i < 8; i++) tx_frame.data[i] = i;

    printf("\nDefault Task Initialized \r\n");

    /* Infinite loop */
    for (;;)
    {
        canstat = MCP_readRegister(MCP_CANSTAT);  // 0x0E
        mode = canstat & 0xE0;  // Mask bits 7:5

        if (mode == CANCTRL_REQOP_NORMAL) {
			CAN_Error tx_result = MCP_sendMessage(&tx_frame);
			printf("TX Result: %d\n\r", tx_result);

			if (MCP_checkReceive()) {
				if (MCP_readMessage(&rx_frame) == ERROR_OK) {
					printf("Received: ");
					printf("CAN ID: 0x%03X", rx_frame.can_id & CAN_SFF_MASK);

					printf("  DLC: %d  Data:", rx_frame.can_dlc);
					for (int i = 0; i < rx_frame.can_dlc; i++) {
					    printf(" %02X", rx_frame.data[i]);
					}
					printf("\r\n");
				}
			} else {
				printf("Received Nothing\r\n");
			}
        }

        osDelay(100);
    }
  /* USER CODE END 5 */
}








/** @brief circular receive buffer size used in HAL_UART_RxCpltCallback, and UART_RX_Task */
#define RX_BUF_SIZE 256

/** @brief circular receive buffer used in HAL_UART_RxCpltCallback and UART_RX_Task */
static uint8_t rx_buf[RX_BUF_SIZE];

/** @brief head index to read bytes out of circular receive buffer used in HAL_UART_RxCpltCallback and UART_RX_Task */
static volatile size_t rx_head = 0;

/** @brief tail index to write bytes into circular receive buffer used in HAL_UART_RxCpltCallback and UART_RX_Task */
static volatile size_t rx_tail = 0;

/** @brief temporary char receive buffer in HAL_UART_RxCpltCallback (called every UART3 interrupt) necessary for UART_RX_Task */
static uint8_t rx_char;

/**
 * @brief UART receive complete interrupt callback.
 * @details
 * Called automatically by HAL when a byte is received through UART
 * configured with interrupt mode. If the interrupt source is USART3,
 * the received byte is stored in the circular receive buffer (`rx_buf`)
 * at the current tail position, and the tail index is advanced with
 * wrap-around. The UART is then re-armed to receive the next byte.
 *
 * @param[in] huart Pointer to the UART handle that triggered the interrupt.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        // Store received char in ring buffer
        rx_buf[rx_tail] = rx_char;
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        // Re-arm UART to receive next char
        HAL_UART_Receive_IT(huart, &rx_char, 1);
    }
}

/* USER CODE BEGIN Header_start_UART_RX_Task */

/**
* @brief Function implementing the UART_RX_Task thread.
* @details
* This task receives UART packages using interrupts, assembles them into lines,
* and parses each complete line into motion control variables for the InputData
* struct instance and PID gain settings in the PIDConfig struct instance. The
* received lines must contain 30 space-separated numbers in the expected order.
* Successfully parsed values are sent to other tasks through determined queues.
* If parsing fails or a buffer overflow occurs, the line is discarded.
* The task runs continuously with a 5ms (arbitrary) delay to allow other tasks to run.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_start_UART_RX_Task */
void start_UART_RX_Task(void *argument)
{
  /* USER CODE BEGIN start_UART_RX_Task */


	char line_buf[RX_BUF_SIZE] = {0};
	uint16_t line_index = 0;
	InputData data = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0f,0.0f,0.0f};
	PIDConfig kpids = {
		.x_pid = {0.0f, 0.0f, 0.0f},
		.y_pid = {0.0f, 0.0f, 0.0f},
		.phi_pid = {0.0f, 0.0f, 0.0f},
		.u_pid = {
			{0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f}
		}
	};

	// Start the first interrupt reception
	HAL_UART_Receive_IT(&huart3, &rx_char, 1);
	/* Infinite loop */
	for (;;) {
//		printf("alive2\r\n");
		// Check if data is available
		while (rx_head != rx_tail) {
			uint8_t byte = rx_buf[rx_head];
			rx_head = (rx_head + 1) % RX_BUF_SIZE;

			// End of line detected
			if (byte == '\n' || byte == '\r') {
				if (line_index > 0) {
					line_buf[line_index] = '\0'; // Null-terminate
//					printf("📥 Full line received: \"%s\"\r\n", line_buf);

					// Optional: parse float data
					if (sscanf(line_buf, "%lf %lf %lf %lf %lf %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
					           &data.x_desired, &data.y_desired,
					           &data.phi_end, &data.d, &data.r,
							   &data.u1_desired, &data.u2_desired, &data.u3_desired, &data.u4_desired,
					           &kpids.x_pid.Kp, &kpids.x_pid.Ki, &kpids.x_pid.Kd,
					           &kpids.y_pid.Kp, &kpids.y_pid.Ki, &kpids.y_pid.Kd,
					           &kpids.phi_pid.Kp, &kpids.phi_pid.Ki, &kpids.phi_pid.Kd,
					           &kpids.u_pid[0].Kp, &kpids.u_pid[0].Ki, &kpids.u_pid[0].Kd,
							   &kpids.u_pid[1].Kp, &kpids.u_pid[1].Ki, &kpids.u_pid[1].Kd,
							   &kpids.u_pid[2].Kp, &kpids.u_pid[2].Ki, &kpids.u_pid[2].Kd,
							   &kpids.u_pid[3].Kp, &kpids.u_pid[3].Ki, &kpids.u_pid[3].Kd) == 30)
					{

						osMessageQueuePut(UART2KPIDs_QueueHandle, &kpids, 0, 0);
						osMessageQueuePut(kpids_UART_TX_QueueHandle, &kpids, 0, 0);
						osMessageQueuePut(UART_QueueHandle, &data, 0, 0);
						osMessageQueuePut(UART2CtrlTsk_QueueHandle, &data, 0, 0);
//						printf("✅ Parsed: x=%.2f y=%.2f phi=%.2f d=%.2f r=%.2f\r\n",
//							   data.x_desired, data.y_desired, data.phi_end, data.d, data.r);
					} else {
						printf("❌ Failed to parse: \"%s\"\r\n", line_buf);
					}
				}
				line_index = 0;
				memset(line_buf, 0, sizeof(line_buf));
			}
			else if (line_index < RX_BUF_SIZE - 1) {
				line_buf[line_index++] = byte;
			} else {
				// Overflow safety
				line_index = 0;
				memset(line_buf, 0, sizeof(line_buf));
				printf("⚠️ Line buffer overflowed and reset.\r\n");
			}
		}

		osDelay(5); // Cooperative multitasking
	}

  /* USER CODE END start_UART_RX_Task */
}


/* USER CODE BEGIN Header_Start_UART_TX_Task */
/**
* @brief UART_TX_Task task that formats and transmits a telemetry line over UART.
* @details
* Initializes local state (IMU, encoders, errors, time, odometry, control outputs)
* and PID gains, then runs a periodic loop that:
*  1) Reads the latest inputs from message queues
*     (UART_QueueHandle = received desired setpoints, kpids_UART_TX_QueueHandle = received PID gains,
*      CtrlTsk_QueueHandle = telemetry). if a queue times out, previous values are kept.
*  2) Prints a single line with comma-separated values in the format variable=value, printf function
*     is retargeted to UART to send the ~60 variables:
*     desired setpoints, IMU angles, encoder counts & wheel speeds, inertial velocities,
*     odometry pose & errors, per-wheel control errors, controller outputs (ẋ, ẏ, φ̇, u[0..3]),
*     timestamps, PWM duty cycles, and PID gains for x, y, φ, and each wheel.
*  3) Has ~10ms osDelay to target ~100 Hz telemetry
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_TX_Task */
void Start_UART_TX_Task(void *argument)
{
  /* USER CODE BEGIN Start_UART_TX_Task */
	  InputData data = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0f,0.0f,0.0f};
	  CtrlTsk_Data CtrlTsk_data;
	  IMUData      *imu      = &CtrlTsk_data.imu;
	  EncoderData  *enc      = &CtrlTsk_data.encoders;
	  Errors       *err      = &CtrlTsk_data.error;
	  TimeState    *ts       = &CtrlTsk_data.time;
	  OdomData     *odom     = &CtrlTsk_data.odom;
	  CtrlOutData  *ctrl_out = &CtrlTsk_data.ctrl;

	  imu->yaw = 0.0;  imu->roll = 0.0;  imu->pitch = 0.0;

	  for (int i = 0; i < 4; i++) {
	      enc->cnt_vals[i] = 0;  enc->angleVals[i] = 0.0f;  enc->omegaVals[i] = 0.0;
	  }

	  err->err_x = 0.0;  err->err_y = 0.0;  err->err_phi = 0.0;
	  for (int i = 0; i < 4; i++) {
	      err->u_errs[i] = 0.0;
	  }
	  ts->current = 1;  ts->previous = 0;  ts->delta = osKernelGetTickCount();  ts->print_prev = 1;

	  odom->x_pos = 0.0;  odom->y_pos = 0.0;  odom->phi = 0.0;

	  ctrl_out->x_dot = 4.0;  ctrl_out->y_dot = 6.0;  ctrl_out->phi_dot = 0.0;

	  for (int i = 0; i < 4; i++) {
	      ctrl_out->PWM_vals[i] = 0.0;  ctrl_out->dutyCycles[i] = 0;  ctrl_out->M_dirs[i] = 0;
	  }

	  PIDConfig kpids = {
	  	.x_pid = {0.0f, 0.0f, 0.0f},
	  	.y_pid = {0.0f, 0.0f, 0.0f},
	  	.phi_pid = {0.0f, 0.0f, 0.0f},
	  	.u_pid = {
	  		{0.0f, 0.0f, 0.0f},
	  		{0.0f, 0.0f, 0.0f},
	  		{0.0f, 0.0f, 0.0f},
	  		{0.0f, 0.0f, 0.0f}
	  	}
	    };

	    PIDGains *xPID_K   = &kpids.x_pid;
	    PIDGains *yPID_K   = &kpids.y_pid;
	    PIDGains *phiPID_K = &kpids.phi_pid;
	    PIDGains *uPID_K   = kpids.u_pid;

//	    printf("alive\r\n");

  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
	osMessageQueueGet(UART_QueueHandle, &data, NULL, 100);
	osMessageQueueGet(kpids_UART_TX_QueueHandle, &kpids, NULL, 100);
	osMessageQueueGet(CtrlTsk_QueueHandle, &CtrlTsk_data, NULL, 100);
	// ~60 variables sent
	printf("x_desired=%lf,y_desired=%lf,phi_desired=%lf,d=%lf,r=%lf,"
			"roll=%lf,pitch=%lf,yaw=%lf,"
			"TIM1=%d,TIM2=%d,TIM4=%d,TIM8=%d,"
			"Enc_Wheel_Omega1=%.6f,Enc_Wheel_Omega2=%.6f,Enc_Wheel_Omega3=%.6f,Enc_Wheel_Omega4=%.6f,"
			"Inertial_ang_vel_calc=%.6f,Inertial_x_vel_calc=%.6f,Inertial_y_vel_calc=%.6f,"
			"ODOM_phi=%.3f,ODOM_x_pos=%.3f,ODOM_y_pos=%.3f,ODOM_Err_x=%0.3lf,ODOM_Err_y=%0.3lf,ODOM_Err_phi=%0.3lf,"
			"U_Err_1=%0.3lf,U_Err_2=%0.3lf,U_Err_3=%0.3lf,U_Err_4=%0.3lf,"
			"Ctrl_Inertial_x_dot=%.3f,Ctrl_Inertial_y_dot=%.3f,Ctrl_Inertial_phi_dot=%.3f,Ctrl_necc_u1=%.3f,Ctrl_necc_u2=%.3f,Ctrl_necc_u3=%.3f,Ctrl_necc_u4=%.3f,"
			"ts_current=%lu,ts_previous=%lu,ts_delta=%lu,"
			"Ctrl_duty_u1=%u,Ctrl_duty_u2=%u,Ctrl_duty_u3=%u,Ctrl_duty_u4=%u,"
			"xKp=%.3f,xKi=%.3f,xKd=%.3f,"
			"yKp=%.3f,yKi=%.3f,yKd=%.3f,"
			"phiKp=%.3f,phiKi=%.3f,phiKd=%.3f,"
			"u0Kp=%.3f,u0Ki=%.3f,u0Kd=%.3f,"
			"u1Kp=%.3f,u1Ki=%.3f,u1Kd=%.3f,"
			"u2Kp=%.3f,u2Ki=%.3f,u2Kd=%.3f,"
			"u3Kp=%.3f,u3Ki=%.3f,u3Kd=%.3f\r\n",
			data.x_desired, data.y_desired, data.phi_end, data.d, data.r,
			imu->roll, imu->pitch, imu->yaw,
			enc->cnt_vals[0], enc->cnt_vals[1], enc->cnt_vals[2], enc->cnt_vals[3],
			enc->omegaVals[0], enc->omegaVals[1], enc->omegaVals[2], enc->omegaVals[3],
			odom->q_dot[0],odom->q_dot[1],odom->q_dot[2], odom->phi,odom->x_pos,odom->y_pos, err->err_x,
			err->err_y, err->err_phi,
			err->u_errs[0],err->u_errs[1],err->u_errs[2],err->u_errs[3],
			ctrl_out->x_dot, ctrl_out->y_dot, ctrl_out->phi_dot, ctrl_out->u[0], ctrl_out->u[1], ctrl_out->u[2], ctrl_out->u[3],
			ts->current, ts->print_prev, ts->delta,
			ctrl_out->dutyCycles[0], ctrl_out->dutyCycles[1], ctrl_out->dutyCycles[2], ctrl_out->dutyCycles[3],
			xPID_K->Kp, xPID_K->Ki, xPID_K->Kd,
			yPID_K->Kp, yPID_K->Ki, yPID_K->Kd,
			phiPID_K->Kp, phiPID_K->Ki, phiPID_K->Kd,
			uPID_K[0].Kp, uPID_K[0].Ki, uPID_K[0].Kd,
			uPID_K[1].Kp, uPID_K[1].Ki, uPID_K[1].Kd,
			uPID_K[2].Kp, uPID_K[2].Ki, uPID_K[2].Kd,
			uPID_K[3].Kp, uPID_K[3].Ki, uPID_K[3].Kd);

    osDelay(9);
  }
  /* USER CODE END Start_UART_TX_Task */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief ControlTask with control loop for a 4‑wheel omni base (read→compute→actuate→publish).
* @details
* Runs a periodic feedback loop that:
*  1) **Sensing:** Reads encoder counters (TIM 1/4/8/2), computes delta counts with wrap‑around
*     via `computeDeltaCNT()`, updates per‑wheel angles, and computes wheel speeds (RPM)
*     with a first‑order low‑pass filter (`alpha = dt/(tau+dt)`). Time step `dt` is derived
*     from CMSIS tick deltas (`osKernelGetTickCount()`). Tick Hz set to 1kHz in FreeRTOSConfig.h
*  2) **Inputs:** Receives desired motion/pose (`UART2CtrlTsk_QueueHandle`) and PID gains
*     (`UART2KPIDs_QueueHandle`). If a queue times out, previous values are retained.
*  3) **Outer control (pose → wheel targets):**
*     - **Pose mode** (`velocity_ctrl_mode == false`): computes `x_dot`, `y_dot`, `phi_dot`
*       with PID (per‑axis I‑sum and D on error; small deadbands on |err|), then calls
*       `computeNecessaryWheelSpeedsMecanum(phi, x_off, y_off, r, u[4], phi_dot, y_dot, x_dot)`
*       to map body‑frame velocities to wheel targets.
*     - **Velocity mode** (`velocity_ctrl_mode == true`): uses `u[0..3]` directly from UART.
*  4) **Inner control (wheel speed → PWM):** Per wheel PID on `u[i] − ω[i]` with integral
*     anti‑windup clamp, small‑error I reset (`|u_err| < u_err_trshld`), sign‑based direction,
*     dead‑zone compensation near zero, and saturation to `[−max_duty, +max_duty]`. Updates
*     GPIO H‑bridge directions and PWM compare registers:
*     `TIM 5/12/14/15` channels.
*  5) **Odometry:** Converts measured wheel speeds to global rates with
*     `globalSpeedsFromUMecanum(phi, x_off, y_off, r, u[4], q_dot[3])`, then integrates
*     `q_dot` to update `odom.{phi, x_pos, y_pos}` and wraps `phi ∈ [−π, π]`.
*  6) **Publish:** Sends the `CtrlTsk_Data` to UART_TX_Tsak through `CtrlTsk_QueueHandle`
*     for telemetry/monitoring.
*
* Constants define encoder resolution (`R_counts`), degrees/radians per count, PID thresholds,
* PWM limits, and filter constant `tau`. Supports a lightweight plant **simulation mode**
* (when `simulation_on == true`) that drives ω toward `u` with a first‑order model.
*
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  int16_t deltaEncCounts[4] = {0,0,0,0};

  float PI = 3.141592;

//  const float R_counts = 17380.0; // MOTORES GRANDES
  const float R_counts = 424.0; // MOTORES CHICOS
  const float Degs_per_count = 360.0 / R_counts;
  const float Rads_per_count = 2*PI / R_counts;

  double u_err_trshld = 0.1;

//  float PID_K_oneM[3] = {0.5,0.003,0.0};
//  float xPID_K[3] = {0.5,0.003,0.0};
//  float yPID_K[3] = {0.5,0.003,0.0};
//  float phiPID_K[3] = {0.5,0.003,0.0};
//  float uPID_K[4][3] = {{1,0,0},{1,0,0},{1,0,0},{0,0,0}};

//  for (int i = 0; i < 4; i++) {
//	  for (int j = 0; j < 3; j++) {
//		  uPID_K[i][j] = PID_K_oneM[j];
//	  }
//  }

  float xTrshld = 0.01; // Allowed error cm
  float yTrshld = 0.01; // Allowed error cm
  float phiTrshld = 0.01; // Allowed error radians

  double sumKI_x = 0;
  double sumKI_y = 0;
  double sumKI_phi = 0;
  double sumKI_u_errs[4] = {0,0,0,0};

  double dt = 0;

//  bno055_vector_t BNO055_EulerVector = bno055_getVectorEuler();

  int16_t deadzone_duty_lim = 1000;
  int16_t max_duty_cycle = 19999;

  InputData data = {0,0,0,1,1,0,0,0,0};
  CtrlTsk_Data CtrlTsk_data;
  IMUData      *imu      = &CtrlTsk_data.imu;
  EncoderData  *enc      = &CtrlTsk_data.encoders;
  Errors       *err      = &CtrlTsk_data.error;
  TimeState    *ts       = &CtrlTsk_data.time;
  OdomData     *odom     = &CtrlTsk_data.odom;
  CtrlOutData  *ctrl_out = &CtrlTsk_data.ctrl;

  Errors  prevErr      = {
	.err_phi = 0.0,
    .err_x = 0.0,
	.err_y = 0.0,
	.u_errs = {0.0,0.0,0.0,0.0}
  };

  PIDConfig kpids = {
	.x_pid = {0.0f, 0.0f, 0.0f},
	.y_pid = {0.0f, 0.0f, 0.0f},
	.phi_pid = {0.0f, 0.0f, 0.0f},
	.u_pid = {
		{0.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 0.0f}
	}
  };

  PIDGains *xPID_K   = &kpids.x_pid;
  PIDGains *yPID_K   = &kpids.y_pid;
  PIDGains *phiPID_K = &kpids.phi_pid;
  PIDGains *uPID_K   = kpids.u_pid;

  imu->yaw = 0.0;  imu->roll = 0.0;  imu->pitch = 0.0;

  for (int i = 0; i < 4; i++) {
      enc->cnt_vals[i] = 0;  enc->prevcnt_vals[i] = 0;  enc->angleVals[i] = 0.0f;  enc->omegaVals[i] = 0.0;
  }

  err->err_x = 0.0;  err->err_y = 0.0;  err->err_phi = 0.0;
  for (int i = 0; i < 4; i++) {
	  err->u_errs[i] = 0.0;
  }
  ts->current = 1;  ts->previous = 0;  ts->delta = osKernelGetTickCount();  ts->print_prev = 1;

  odom->x_pos = 0.0;  odom->y_pos = 0.0;  odom->phi = 0.0; odom->q_dot[0] = 0; odom->q_dot[1] = 0; odom->q_dot[2] = 0;

  ctrl_out->x_dot = 4.0;  ctrl_out->y_dot = 6.0;  ctrl_out->phi_dot = 0.0;

  for (int i = 0; i < 4; i++) {
      ctrl_out->PWM_vals[i] = 0.0;  ctrl_out->dutyCycles[i] = 0;  ctrl_out->M_dirs[i] = 0;  ctrl_out->u[i] = 0.0;
  }

  osStatus_t queue_status;

  bool simulation_on = false;

  bool velocity_ctrl_mode = true;

  double max_integral_action_val = 999999999999999;

  float tau = 0.05; //Low pass filter time constant (time it takes to reach 63.2% of the new velocity reading value
  float alpha = 0.0; // Low pass filter alpha value = dt / (tau + dt)

  double prevERR = 0.0; // temp prev error for U pids

  /* Infinite loop */
  for(;;)
  {

    osDelay(10);

/*------------------------------------------------------------------------*/
/******* 1. Get angular position and velocity with encoders & IMU *********/
    if (!simulation_on) {
		enc->cnt_vals[0] = TIM1->CNT; enc->cnt_vals[1] = TIM4->CNT; enc->cnt_vals[2] = TIM8->CNT; enc->cnt_vals[3] = TIM2->CNT;

//		BNO055_EulerVector = bno055_getVectorEuler();
//		imu->yaw = BNO055_EulerVector.x;  imu->roll = BNO055_EulerVector.y;  imu->pitch = BNO055_EulerVector.z;

		for (int i = 0; i < 4; i++) {
		    // Calculate counter difference considering wrap-around
		    deltaEncCounts[i] = computeDeltaCNT(enc->cnt_vals[i], enc->prevcnt_vals[i]);

		    // Save current count for next iteration
		    enc->prevcnt_vals[i] = enc->cnt_vals[i];

		    // Update total angle value
		    enc->angleVals[i] += deltaEncCounts[i] * Degs_per_count;
		}

		// Calculate velocities
		ts->print_prev = ts->previous;
		ts->current = osKernelGetTickCount();
		ts->delta = ts->current - ts->previous;
		ts->previous = ts->current;
		dt = ts->delta / 1000.0f;  // Convert ms to seconds
		//	printf(" currentTime: %lu prevTime: %lu deltaTime: %lu", currentTime, prevTime, deltaTime);

		//******* GET VELOCITIES WITH FIRST ORDER FILTER****//
		alpha = dt / (tau + dt);
		for (uint8_t i = 0; i < 4; i++){
			double rawRPM = (deltaEncCounts[i] * Rads_per_count /  (ts->delta / 1000.0f) / (2*PI));
			enc->omegaVals[i] = alpha * rawRPM + (1.0 - alpha) * enc->omegaVals[i];
		}


    } else {
//		BNO055_EulerVector = bno055_getVectorEuler();
//		imu->yaw = BNO055_EulerVector.x;  imu->roll = BNO055_EulerVector.y;  imu->pitch = BNO055_EulerVector.z;

    	ts->print_prev = ts->previous;
		ts->current = osKernelGetTickCount();
		ts->delta = ts->current - ts->previous;
		ts->previous = ts->current;

		// Simple first-order simulation model: omega approaches u with time
		float tau_2 = 0.1f;  // time constant for motor response
		float alpha_2 = ts->delta / (1000.0f * tau_2 + ts->delta);  // convert delta to seconds

    	for (int i = 0; i < 4; i++) {
    	    enc->omegaVals[i] = (1 - alpha_2) * enc->omegaVals[i] + alpha_2 * ctrl_out->u[i];

    	    // Update total angle by integrating omega
    	    float dt_2 = ts->delta / 1000.0f;  // ms to s
    	    enc->angleVals[i] += enc->omegaVals[i] * dt_2;

    	    // Simulate counts from angle
    	    enc->cnt_vals[i] = (uint16_t)(enc->angleVals[i] / Degs_per_count) % 65536;
    	    deltaEncCounts[i] = computeDeltaCNT(enc->cnt_vals[i], enc->prevcnt_vals[i]);
    	    enc->prevcnt_vals[i] = enc->cnt_vals[i];
    	}
    }
/***************************************************************************/
/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
/**************** 2. Receive desired pose/pidKs from UART *****************/

	osMessageQueueGet(UART2CtrlTsk_QueueHandle, &data, NULL, 100);
	osMessageQueueGet(UART2KPIDs_QueueHandle, &kpids, NULL, 100);

/**************************************************************************/
/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
/************************** 3. Compute errors *****************************/
	prevErr->err_x = err->err_x;
	prevErr->err_y = err->err_y;
	prevErr->err_phi = err->err_phi;

	err->err_x = data.x_desired - odom->x_pos;
	err->err_y = data.y_desired - odom->y_pos;
	//phi_desired = atan2(Err_y, Err_x); /*<-- Unnecessary? ony for diff drive?*/

	/* POTENTIALLY READ IMU HERE (TO UPDATE PHI)*/
	err->err_phi = data.phi_end - odom->phi;
//	printf("Err_x: %0.3lf  Err_y: %0.3lf  Err_phi: %0.3lf \n", Err_x, Err_y, Err_phi);
/**************************************************************************/
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/***************************** 4. POSE PIDs *******************************/
	if (!velocity_ctrl_mode) {

		if (abs(err->err_x) > xTrshld){
			sumKI_x += err->err_x * ts->delta;
			ctrl_out->x_dot = xPID_K->Kp*err->err_x + xPID_K->Ki*sumKI_x + xPID_K->Kd * ((err->err_x - prevErr->err_x)/ ts->delta);
			ctrl_out->phi_dot = 0;
		}

		if (abs(err->err_y) > yTrshld){
			sumKI_y += err->err_y * ts->delta;
			ctrl_out->y_dot = yPID_K->Kp*err->err_y + yPID_K->Ki*sumKI_y + yPID_K->Kd * ((err->err_y - prevErr->err_y) / ts->delta);
			ctrl_out->phi_dot = 0;
		}

		if (abs(err->err_x) <= xTrshld && abs(err->err_y) <= yTrshld) {
			ctrl_out->x_dot = 0;
			ctrl_out->y_dot = 0;

			if (abs(err->err_phi) > phiTrshld){
				sumKI_phi += err->err_phi * ts->delta;
				ctrl_out->phi_dot = phiPID_K->Kp*err->err_phi + phiPID_K->Ki*sumKI_phi + phiPID_K->Kd * ((err->err_phi - prevErr->err_phi) / ts->delta);
			}
		}

		if (abs(err->err_x) <= xTrshld && abs(err->err_y) <= yTrshld && abs(err->err_phi) <= phiTrshld){

			ctrl_out->x_dot = 0;
			ctrl_out->y_dot = 0;
			ctrl_out->phi_dot = 0;
			sumKI_x = 0; sumKI_y = 0; sumKI_phi = 0;
		}

	//	printf("Calling computeWheelSpeeds(phi=%.2f, d=%.2f, r=%.2f, phi_dot=%.2f, y_dot=%.2f, x_dot=%.2f)\r\n",
	//	       0.0, 1.0, 1.0, 0.0, 3.0, 6.0);

	//	computeNecessaryWheelSpeeds(0.0, data.d, data.r, ctrl_out->u, 0.0, 3.0, 6.0);
		computeNecessaryWheelSpeedsMecanum(odom->phi, data.d, data.r, ctrl_out->u, ctrl_out->phi_dot, ctrl_out->y_dot, ctrl_out->x_dot);

	}
	else {
		ctrl_out->u[0] = data.u1_desired;
		ctrl_out->u[1] = data.u2_desired;
		ctrl_out->u[2] = data.u3_desired;
		ctrl_out->u[3] = data.u4_desired;
	}

/**************************************************************************/
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/***************************** 6. MOTOR PIDs *******************************/

	for (int i = 0; i < 4; i++) {
		prevERR = err->u_errs[i];
		err->u_errs[i] = ctrl_out->u[i] - enc->omegaVals[i];
		sumKI_u_errs[i] += err->u_errs[i] * ts->delta;
		if (sumKI_u_errs[i] > max_integral_action_val) sumKI_u_errs[i] = max_integral_action_val;
		if (sumKI_u_errs[i] < -max_integral_action_val) sumKI_u_errs[i] = -max_integral_action_val;

		ctrl_out->PWM_vals[i] = uPID_K[i].Kp*err->u_errs[i] + uPID_K[i].Ki*sumKI_u_errs[i] + uPID_K[i].Kd * ((err->u_errs[i] - prevERR) / ts->delta);

		if (abs(err->u_errs[i]) < u_err_trshld){
			sumKI_u_errs[i] = 0; // Reset Integral part if error small
		}

		if (fabs(ctrl_out->u[i]) < 0.005) {
			ctrl_out->PWM_vals[i] = 0;
			ctrl_out->dutyCycles[i] = (uint16_t)0;
		}
		else {
			if (ctrl_out->PWM_vals[i] < 0){
				ctrl_out->M_dirs[i] = 1;
				if (ctrl_out->PWM_vals[i] < 0 && ctrl_out->PWM_vals[i] > -deadzone_duty_lim) ctrl_out->PWM_vals[i] = -deadzone_duty_lim;
				else if (ctrl_out->PWM_vals[i] < -max_duty_cycle) ctrl_out->PWM_vals[i] = -max_duty_cycle;
				ctrl_out->dutyCycles[i] = (uint16_t)(ctrl_out->PWM_vals[i]*-1);
			}
			else if (ctrl_out->PWM_vals[i] >= 0){
				ctrl_out->M_dirs[i] = 0;
				if (ctrl_out->PWM_vals[i] > 0 && ctrl_out->PWM_vals[i] < deadzone_duty_lim) ctrl_out->PWM_vals[i] = deadzone_duty_lim;
				else if (ctrl_out->PWM_vals[i] > max_duty_cycle) ctrl_out->PWM_vals[i] = max_duty_cycle;
				ctrl_out->dutyCycles[i] = (uint16_t)(ctrl_out->PWM_vals[i]);
			}
		}
	}

	setMotorDirection(GPIOD, 4, 5, ctrl_out->M_dirs[0]);
	setMotorDirection(GPIOD, 6, 7, ctrl_out->M_dirs[1]);
	setMotorDirection(GPIOE, 2, 4, ctrl_out->M_dirs[2]);
	setMotorDirection(GPIOE, 3, 6, ctrl_out->M_dirs[3]);

	__HAL_TIM_SET_COMPARE(&htim5 , TIM_CHANNEL_1, ctrl_out->dutyCycles[0]);  // Set duty cycle
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, ctrl_out->dutyCycles[1]);  // Set duty cycle
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ctrl_out->dutyCycles[2]);  // Set duty cycle
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctrl_out->dutyCycles[3]);  // Set duty cycle

/**************************************************************************/
/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
/***************************** 7. ODOMETRY ********************************/

	globalSpeedsFromUMecanum(odom->phi, data.d, data.r, enc->omegaVals, odom->q_dot); // q_dot = {phi_dot, x_dot, y_dot}
//	globalSpeedsFromU(odom->phi, data.d, data.r, ctrl_out->u, odom->q_dot); // q_dot = {phi_dot, x_dot, y_dot}

	odom->phi    += odom->q_dot[0] * dt;         // Integrated angular velocity
	odom->x_pos  += odom->q_dot[1] * dt;         // Integrated x velocity (global)
	odom->y_pos  += odom->q_dot[2] * dt;         // Integrated y velocity (global)

	odom->phi = (odom->phi > PI) ? (odom->phi - 2 * PI) : (odom->phi < -PI) ? (odom->phi + 2 * PI) : odom->phi; // 0 <= phi < 2PI


	/*ADD FIRST ORDER FILTER?*/
//	q_dot_filtered[i] = alpha * q_dot[i] + (1 - alpha) * q_dot_filtered[i]; // for i = 0 to 2

/**************************************************************************/
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/************************ 8. SEND RELEVANT DATA ***************************/

//
//	printf(" currentTime: %lu prevTime: %lu deltaTime: %lu 	dt: %0.9lf     ", currentTime, printf_prevTime, deltaTime, dt);
//

	queue_status = osMessageQueuePut(CtrlTsk_QueueHandle, &CtrlTsk_data, 0, 0);

	if (queue_status != osOK) {
	    printf("❌ Failed to enqueue message! Error code: %d\r\n", queue_status);
	}
  }
  /* USER CODE END StartControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
