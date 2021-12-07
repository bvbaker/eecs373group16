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

// I2C Color Sensor Defines
#define COLOR_I2C_ADDR 		(0x29 << 1)

#define COLOR_ENABLE_SAD 	(0x00)
#define COLOR_STATUS_SAD 	(0x13)

#define COLOR_COMMAND_BIT	(0x80)

#define COLOR_DATA_C 		(0x14)
#define COLOR_DATA_R		(0x16)
#define COLOR_DATA_G		(0x18)
#define COLOR_DATA_B		(0x1A)

// I2C Display Defines
#define DISPLAY_I2C_ADDR	(0x50)
#define DISPLAY_WIDTH		(20)

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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

	// Up Button

	up_pressed = 1;

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

	// Down Button

	down_pressed = 1;

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

	// Menu/Back Button

	menu_pressed = 1;

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

	// OK Button

	ok_pressed = 1;

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

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

void color_init() {
	uint8_t buffer[2];
	HAL_StatusTypeDef ret;

	buffer[0] = COLOR_COMMAND_BIT | COLOR_ENABLE_SAD;
	buffer[1] = 0b1; // PON
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);

	buffer[0] = (1 << 7) | COLOR_ENABLE_SAD;
	buffer[1] = 0b11; // AEN and PON
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);
}

void color_off() {
	uint8_t buffer[2];
	HAL_StatusTypeDef ret;

	buffer[0] = COLOR_COMMAND_BIT | COLOR_ENABLE_SAD;
	buffer[1] = 0b0; // PON
	ret = HAL_I2C_Master_Transmit(&hi2c1, (COLOR_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
}

struct RelativeColorType color_read_perc() {
	struct RelativeColorType rel_col;
	struct ColorType col = color_read();
}

struct ColorType color_read_rgbc() {
	struct ColorType col;
	col.r = color_read('r');
	col.g = color_read('g');
	col.b = color_read('b');
	col.c = color_read('c');
	return col;
}

uint16_t color_read(char color) {
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

		HAL_Delay(5);

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

	HAL_Delay(5);

	return (uint16_t) ((buffer[0] & 0xFF) | buffer[1] << 8);

}

void display_test() {
	uint8_t buffer[DISPLAY_WIDTH];
	char string[DISPLAY_WIDTH];
	HAL_StatusTypeDef ret;

	display_clear(hi2c1);



	for (int i = 0; i < DISPLAY_WIDTH; i++) {
//		string[i] = 'a' + i;
		string[i] = 'a';
	}

//	string_to_uint8_t(string, buffer, DISPLAY_WIDTH);

	// set cursor

	buffer[0] = 0xFE;
	buffer[1] = 0x45;
	buffer[2] = 0;

	// write data
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);

	string_to_uint8_t("hello", buffer, 5);
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 5, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);

	buffer[0] = 0xFE;
	buffer[1] = 0x45;
	buffer[2] = DISPLAY_WIDTH;

	// write data
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);


	for (int i = 0; i < DISPLAY_WIDTH; i++) {
		string[i] = 'a' + i;
	}

	string_to_uint8_t(string, buffer, DISPLAY_WIDTH);

	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 20, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);

//	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, DISPLAY_WIDTH, HAL_MAX_DELAY);
//	if ( ret != HAL_OK ) {
//		while(1);
//	}
}

void display_color_debug() {
	struct ColorType col = color_read_rgbc();

	char red_line[14], green_line[20], blue_line[20];

	strcpy(red_line, "RED   : ");//__.__%");
	strcpy(red_line, "GREEN : ");//__.__%");
	strcpy(red_line, "BLUE  : ");//__.__%");

	display_print_line("Relative Color:", 15, 0);

//	display_print_line();
}

void display_print_line(char* str, int len, int line) {
	display_set_cursor_line(line);

	uint8_t buffer[DISPLAY_WIDTH];
	HAL_StatusTypeDef ret;

	string_to_uint8_t(str, buffer, len);

	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, len, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);
}

// line can be any number, will go to the modulo of that number
void display_set_cursor_line(int line) {
	uint8_t buffer[3];
	HAL_StatusTypeDef ret;

	buffer[0] = 0xFE;
	buffer[1] = 0x45;
	switch (line % 4) {
	case (0):
		buffer[2] = 0x00;
		break;
	case (1):
		buffer[2] = 0x40;
		break;
	case (2):
		buffer[2] = 0x14;
		break;
	case (3):
		buffer[2] = 0x54;
		break;
	default:
		buffer[2] = 0x00;
		break;
	}


	// write data
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	HAL_Delay(5);
}

void display_clear() {
	uint8_t buffer[2] = {0xFE, 0x51};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
	HAL_Delay(5); // clear takes 1.5ms
}

void display_on() {
	uint8_t buffer[2] = {0xFE, 0x41};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
	HAL_Delay(5);
}

void display_set_brightness(uint8_t brightness_in) {
	uint8_t brightness = brightness_in;
	if (8 < brightness)
		brightness = 8;
	uint8_t buffer[3] = {0xFE, 0x53, brightness};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
	HAL_Delay(5);
}

void display_set_contrast(uint8_t contrast_in) {
	uint8_t brightness = contrast_in;
	if (50 < brightness)
		brightness = 50;
	uint8_t buffer[3] = {0xFE, 0x52, brightness};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
	HAL_Delay(5);
}

void display_off() {

	uint8_t buffer[3] = {0xFE, 0x53, 1};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (DISPLAY_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
	HAL_Delay(5);
}

// timestring is length 5, datestring is length 8
void time_make_string(char* timestring, char* datestring, RTC_TimeTypeDef currTime, RTC_DateTypeDef currDate) {
	// Create Timestring
	int temp = currTime.Hours;
	timestring[0] = '0' + ((temp / 10) % 10);
	timestring[1] = '0' + temp % 10;
	timestring[2] = ':';
	temp = currTime.Minutes;
	timestring[3] = '0' + ((temp / 10) % 10);
	timestring[4] = '0' + temp % 10;

	// Create Datestring
	temp = currDate.Month;
	datestring[0] = '0' + ((temp / 10) % 10);
	datestring[1] = '0' + temp % 10;
	datestring[2] = '-';
	temp = currDate.Date;
	datestring[3] = '0' + ((temp / 10) % 10);
	datestring[4] = '0' + temp % 10;
	datestring[5] = '-';
	temp = currDate.Year;
	datestring[6] = '0' + ((temp / 10) % 10);
	datestring[7] = '0' + temp % 10;
}

void display_init(RTC_TimeTypeDef currTime, RTC_DateTypeDef currDate) {
	  display_on();
	  display_clear();
	  display_set_brightness(4);
	  display_set_contrast(50);
	  display_print_line("Initializing...", 15, 0);
	  char datestring[8];
	  char timestring[5];
	  time_make_string(timestring, datestring, currTime, currDate);
	  display_print_line(timestring, 5, 1);
	  display_print_line(datestring, 8, 2);
}

void string_to_uint8_t(char* str, uint8_t* buff, int len) {
	if (str == NULL) {
		// invalid str array
		while(1);
		return;
	}

	if (buff == NULL) {
		while(1);
		return;
	}

	for (int i = 0; i < len; i++) {
		if (str[i] == 0)
			buff[i] = (uint8_t)' ';
		else
			buff[i] = (uint8_t)str[i];
	}

	return;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
