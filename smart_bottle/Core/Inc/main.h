/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define DISPLAY_WIDTH		(20)

struct MenuItem {
	uint8_t valid;
	char display[DISPLAY_WIDTH - 1];
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
I2C_HandleTypeDef extern hi2c1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Color Functions
void extern color_init();
void extern color_off();
uint16_t extern color_read(char color);

// Button Variables (0 when not pressed, 1 when pressed and not handled)
int extern up_pressed;
int extern down_pressed;
int extern menu_pressed;
int extern ok_pressed;

// Display Functions
void extern display_test();
void extern string_to_uint8_t(char* str, uint8_t* buff, int len);
void extern display_clear();
void extern display_print_line(char* str, int len, int line);
void extern display_set_cursor_line(int line);
void extern display_on();
void extern display_off();
void extern display_set_brightness(uint8_t brightness);
void extern display_init();

// Menu Functions
void extern menu();
void extern display_menu(int menu_idx);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define MAIN_MENU_SIZE (8)
#define NUM_DISPLAY_LINES (4)

// Main Menu Options
#define GUESS_LIQUID 0
#define	ADD_TO_TOTAL 1
#define UPDATE_GOALS 2
#define RESET_DAY 3
#define CHECK_TIME 4
#define CHECK_DAY 5
#define CHECK_WEEK 6
#define RESET_ALL 7
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
