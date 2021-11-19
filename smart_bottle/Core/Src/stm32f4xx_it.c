/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COLOR_I2C_ADDR 		(0x29 << 1)

#define COLOR_ENABLE_SAD 	0x00
#define COLOR_STATUS_SAD 	0x13

#define COLOR_COMMAND_BIT	0x80

#define COLOR_DATA_C 		0x14
#define COLOR_DATA_R		0x16
#define COLOR_DATA_G		0x18
#define COLOR_DATA_B		0x1A

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void color_init(I2C_HandleTypeDef hi2c1) {
	uint8_t buffer[2];
	HAL_StatusTypeDef ret;

	buffer[0] = COLOR_COMMAND_BIT | COLOR_ENABLE_SAD;
	buffer[1] = 0b1; // PON
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	buffer[0] = (1 << 7) | COLOR_ENABLE_SAD;
	buffer[1] = 0b11; // AEN and PON
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
}

void color_off(I2C_HandleTypeDef hi2c1) {
	uint8_t buffer[2];
	HAL_StatusTypeDef ret;

	buffer[0] = COLOR_COMMAND_BIT | COLOR_ENABLE_SAD;
	buffer[1] = 0b0; // PON
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
}

uint16_t color_read(I2C_HandleTypeDef hi2c1, char color) {
	uint8_t buffer[2];
	HAL_StatusTypeDef ret;

	buffer[0] = 0;
	buffer[1] = 0;

	// first, check valid, and don't stop asking until we get a valid response
	while (!buffer[0]) {
		buffer[0] = COLOR_COMMAND_BIT | COLOR_STATUS_SAD; // buffer[0] holds the sub-address

		// read status reg
		ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 1, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
			while(1);
		}

		ret = HAL_I2C_Master_Receive(&hi2c1, (COLOR_I2C_ADDR), buffer, 1, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
			while(1);
		}


		buffer[0] &= 0x01; // check the first bit for rgbc valid
	}

	buffer[0] = COLOR_COMMAND_BIT;

	switch (color) {
	case 'r':
		buffer[0] |= COLOR_DATA_R;
		break;

	case 'g':
		buffer[0] |= COLOR_DATA_G;
		break;

	case 'b':
		buffer[0] |= COLOR_DATA_B;
		break;

	case 'c':
		buffer[0] |= COLOR_DATA_C;
		break;
	default:
//		printf("error: invalid color read");
		return 0;
	}

	// read data
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	return (uint16_t) ((buffer[0] & 0xFF) | buffer[1] << 8);

}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
