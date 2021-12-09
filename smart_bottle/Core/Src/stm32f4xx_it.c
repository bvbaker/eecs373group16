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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Button Defines
#define BUTTON_DELAY		(1000000)

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

// I2C Load Cell Defines
#define LOAD_CELL_I2C_ADDR	(0x2A << 1)
#define LOAD_CELL_PU_CTRL	(0x00)
#define LOAD_CELL_ADC_REG	(0x12)
#define LOAD_CELL_DATA_READY_MASK (1 << 5)

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
	for (volatile int i = 0; i < BUTTON_DELAY; i++);  // plz don't bounce

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
	for (volatile int i = 0; i < BUTTON_DELAY; i++);  // plz don't bounce

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
	for (volatile int i = 0; i < BUTTON_DELAY; i++);  // plz don't bounce

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
	for (volatile int i = 0; i < BUTTON_DELAY; i++);  // plz don't bounce

	ok_pressed = 1;

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void reset_buttons() {
	  menu_pressed = 0;
	  ok_pressed = 0;
	  up_pressed = 0;
	  down_pressed = 0;
}

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

void display_day_summary(struct DayType day_in, RTC_DateTypeDef date_in) {
	display_clear();
	char buffer[20];
	sprintf(buffer, "Consumption for: ");
	display_print_line(buffer, strlen(buffer), 0);
	sprintf(buffer, " %d-%d-%d", date_in.Month, date_in.Date, date_in.Year);
	display_print_line(buffer, strlen(buffer), 1);
	sprintf(buffer, "Sugar: %.0fg Cal: %.0f", day_in.nutrition_total.sugar_g, day_in.nutrition_total.calories);
	display_print_line(buffer, strlen(buffer), 2);
	sprintf(buffer, "Caffeine: %.0fmg", day_in.nutrition_total.caffeine_mg);
	display_print_line(buffer, strlen(buffer), 3);

	reset_buttons();

	while(!menu_pressed && !ok_pressed) {

	}

	display_off();
	return;

}

void display_week_summary() {
//	display_clear();
//	char buffer[20];
//	sprintf(buffer, "Consumption for: ");
//	display_print_line(buffer, strlen(buffer), 0);
//
//	sprintf(buffer, "Tuesday");
//	display_print_line(buffer, strlen(buffer), 1);
//	sprintf(buffer, "Sugar: %dg Cal: %d", day_in.nutrition_total.sugar_g, day_in.nutrition_total.calories);
//	display_print_line(buffer, strlen(buffer), 2);
//	sprintf(buffer, "Caffeine: %dmg", day_in.nutrition_total.caffeine_mg);
//	display_print_line(buffer, strlen(buffer), 3);
//
//	buttons_reset();
//
//	while(!menu_pressed && !ok_pressed) {
//
//	}
//
//	display_off();
//	return;
}

void guess_liquid() {
	struct DrinkType all_drinks[NUM_DRINKS + 2]; // +2 for none and unknown
	struct DrinkType MtnDewRegular;
	// color reading range
	MtnDewRegular.min.r_perc = 34.5;
	MtnDewRegular.min.g_perc = 35.0;
	MtnDewRegular.min.b_perc = 27.5;

	MtnDewRegular.max.r_perc = 36.5;
	MtnDewRegular.max.g_perc = 37.0;
	MtnDewRegular.max.b_perc = 29.5;

	// nutrition data
	MtnDewRegular.serving_size_ml = 591.0;
	MtnDewRegular.nutrition_per_serving.caffeine_mg = 91.0;
	MtnDewRegular.nutrition_per_serving.sugar_g = 77.0;
	MtnDewRegular.nutrition_per_serving.sodium_mg = 105.0;
	MtnDewRegular.nutrition_per_serving.calories = 290.0;
	MtnDewRegular.nutrition_per_serving.carbs_g = 77.0;
	MtnDewRegular.nutrition_per_serving.protein_g = 0.0;
	MtnDewRegular.nutrition_per_serving.fat_g = 0.0;

	MtnDewRegular.already_guessed = 0;

	strcpy(MtnDewRegular.name, "Mountain Dew");

	all_drinks[MTN_DEW_REGULAR] = MtnDewRegular;

	struct DrinkType MtnDewCodeRed;

	// color reading range
	MtnDewCodeRed.max.r_perc = 30.5;
	MtnDewCodeRed.max.g_perc = 37.0;
	MtnDewCodeRed.max.b_perc = 37.0;

	MtnDewCodeRed.min.r_perc = 27.5;
	MtnDewCodeRed.min.g_perc = 34.0;
	MtnDewCodeRed.min.b_perc = 34.0;

	// nutrition data
	MtnDewCodeRed.serving_size_ml = 591.0;
	MtnDewCodeRed.nutrition_per_serving.caffeine_mg = 77.0;
	MtnDewCodeRed.nutrition_per_serving.sugar_g = 76.0;
	MtnDewCodeRed.nutrition_per_serving.sodium_mg = 180.0;
	MtnDewCodeRed.nutrition_per_serving.calories = 280.0;
	MtnDewCodeRed.nutrition_per_serving.carbs_g = 76.0;
	MtnDewCodeRed.nutrition_per_serving.protein_g = 0.0;
	MtnDewCodeRed.nutrition_per_serving.fat_g = 0.0;

	strcpy(MtnDewCodeRed.name, "Mtn Dew Code Red");

	MtnDewCodeRed.already_guessed = 0;

	all_drinks[MTN_DEW_CODE_RED] = MtnDewCodeRed;

	struct DrinkType None = {0};
	// color reading range
	None.max.r_perc = 27.5;
	None.max.g_perc = 37.0;
	None.max.b_perc = 37.0;

	None.min.r_perc = 27.5;
	None.min.g_perc = 34.0;
	None.min.b_perc = 34.0;

	strcpy(None.name, "Water");


	struct DrinkType Unknown = {0};
	strcpy(Unknown.name, "Unknown Drink");

	struct RelativeColorType average;
	average.r_perc = 0.0;
	average.g_perc = 0.0;
	average.b_perc = 0.0;

//	for (int i = 0; i < NUM_COLOR_SAMPLES; i++) {
//		color_in = color_read_percent();
//		average.r_perc += color_in.r_perc / (float)NUM_COLOR_SAMPLES;
//		average.g_perc += color_in.g_perc / (float)NUM_COLOR_SAMPLES;
//		average.b_perc += color_in.b_perc / (float)NUM_COLOR_SAMPLES;
//	}
	average = color_read_percent_average(NUM_COLOR_SAMPLES);

	// go for positive ID

	for (int i = 0; i < NUM_DRINKS + 2; i++) {
		if (guess_within_range(all_drinks[i], average)) {
			all_drinks[i].already_guessed = 1;
			if (display_guess(all_drinks[i])) {
				// measure and add to daily totals
				HAL_Delay(1000);
				float volume_ml = volume_ml_read_avg();
				week_add_measurement(all_drinks[i], volume_ml);
				return;
			}
		}
	}

	// grasp for straws

	for (int i = 0; i < NUM_DRINKS + 2; i++) {
		if (!all_drinks[i].already_guessed) {
			if (display_guess(all_drinks[i])) {
				// measure and add to daily totals
				HAL_Delay(1000);
				float volume_ml = volume_ml_read_avg();
				week_add_measurement(all_drinks[i], volume_ml);
				return;
			}
		}
	}

	display_guess(all_drinks[DRINK_UNKNOWN]);
	HAL_Delay(1000);
	float volume_ml = volume_ml_read_avg();
	week_add_measurement(all_drinks[DRINK_UNKNOWN], volume_ml);
}

float volume_ml_read_avg() {
	float volume_ml = 0.0;

	float height_cm = height_read_cm_avg(NUM_HEIGHT_SAMPLES);
	volume_ml += PI * RADIUS_CM * RADIUS_CM * height_cm;
	volume_ml += VOLUME_OFFSET_ML;

	return volume_ml;
}

struct NutritionType nutrition_accumulate_amount(struct NutritionType accumulator, struct NutritionType addition, float volume_ml, float serving_size_ml) {
	float multiplier = volume_ml / serving_size_ml;

	accumulator.caffeine_mg += addition.caffeine_mg * multiplier;
	accumulator.sugar_g += addition.sugar_g * multiplier;
	accumulator.calories += addition.calories * multiplier;
	accumulator.sodium_mg += addition.sodium_mg * multiplier;
	accumulator.carbs_g += addition.carbs_g * multiplier;
	accumulator.protein_g += addition.protein_g * multiplier;
	accumulator.fat_g += addition.fat_g * multiplier;

	return accumulator;
}

void week_reset_one_day(int index) {
	this_week.day[index].nutrition_total.caffeine_mg = 0;
	this_week.day[index].nutrition_total.sugar_g = 0;
	this_week.day[index].nutrition_total.sodium_mg = 0;
	this_week.day[index].nutrition_total.calories = 0;
	this_week.day[index].nutrition_total.carbs_g = 0;
	this_week.day[index].nutrition_total.protein_g = 0;
	this_week.day[index].nutrition_total.fat_g = 0;
}

void week_reset() {
	for (int i = 0; i < 7; i++) {
		week_reset_one_day(i);
	}
}

struct DrinkType drink_get_empty() {
	struct DrinkType empty_drink;
	empty_drink.serving_size_ml = 1.0;
	strcpy(empty_drink.name, "");
	empty_drink.already_guessed = 0;

	empty_drink.nutrition_per_serving.caffeine_mg = 0;
	empty_drink.nutrition_per_serving.sugar_g = 0;
	empty_drink.nutrition_per_serving.sodium_mg = 0;
	empty_drink.nutrition_per_serving.calories = 0;
	empty_drink.nutrition_per_serving.carbs_g = 0;
	empty_drink.nutrition_per_serving.protein_g = 0;
	empty_drink.nutrition_per_serving.fat_g = 0;

	return empty_drink;
}

void week_add_measurement(struct DrinkType drink_in, float volume_ml) {
	  RTC_TimeTypeDef currTime = {0};
	  RTC_DateTypeDef currDate = {0};
	  HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	  struct Date last_date, curr_date;
	  last_date.d = last_read_date.Date;
	  last_date.m = last_read_date.Month;
	  last_date.y = last_read_date.Year;

	  curr_date.d = currDate.Date;
	  curr_date.m = currDate.Month;
	  curr_date.y = currDate.Year;

	  int curr_weekday_idx = currDate.WeekDay - 1;
	  int last_weekday_idx = last_read_date.WeekDay - 1;

	  int days_since_update = getDifference(last_date, curr_date);

	  // clear all weekdays since the last reading
	  if (days_since_update > 0) {
		  if (days_since_update >= 7) {
			  // clear out the week
			  week_reset();
		  } else {
			  // clear out days between currDate and delete what was at currDate
			  int i = last_weekday_idx;
			  while (1) {
				  week_reset_one_day(i);
				  if (i == curr_weekday_idx) {
					  break;
				  }

				  if (i + 1 < 7) {
					  i++;
				  } else {
					  i = 0;
				  }
			  }
		  }
	  }

	  this_week.day[curr_weekday_idx].nutrition_total =
			  nutrition_accumulate_amount(this_week.day[curr_weekday_idx].nutrition_total, drink_in.nutrition_per_serving, volume_ml, drink_in.serving_size_ml);

}

struct RelativeColorType extern color_read_percent_average(int num_samples) {
	struct RelativeColorType color_in, average;
	average.r_perc = 0;
	average.g_perc = 0;
	average.b_perc = 0;

	for (int i = 0; i < num_samples; i++) {
		color_in = color_read_percent();
		average.r_perc += color_in.r_perc / (float)num_samples;
		average.g_perc += color_in.g_perc / (float)num_samples;
		average.b_perc += color_in.b_perc / (float)num_samples;
	}

	return average;
}

int guess_within_range(struct DrinkType drink, struct RelativeColorType measured) {
	if (drink.min.r_perc < measured.r_perc && measured.r_perc < drink.max.r_perc)
		if (drink.min.g_perc < measured.g_perc && measured.g_perc < drink.max.g_perc)
			if (drink.min.b_perc < measured.b_perc && measured.b_perc < drink.max.b_perc)
				return 1;
	return 0;
}

// Returns 1 if the guess was correct and 0 if not
int display_guess(struct DrinkType guessed_drink) {
	reset_buttons();
	display_clear();

	char buffer[20];
	display_print_line("Is this your drink?", strlen("Is this your drink?"), 0);
	strcpy(buffer, "> ");
	strcat(buffer, guessed_drink.name);
	display_print_line(buffer, strlen(buffer), 2);
	while (1) {
		if (menu_pressed) {
			return 0;
		} else if (ok_pressed) {
			return 1;
		}
	}
}

void clear_string(char* buffer, int size) {
	buffer = memset(buffer, 0, size);
}

void color_make_percent_line(char* buffer, float percent, char color) {
	clear_string(buffer, DISPLAY_WIDTH);
	switch (color) {
	case ('r'):
		sprintf(buffer, "RED   : %5.2f%%", percent);
		break;
	case ('g'):
		sprintf(buffer, "GREEN : %5.2f%%", percent);
		break;
	case ('b'):
		sprintf(buffer, "BLUE  : %5.2f%%", percent);
		break;
	default:
		strcpy(buffer, "OH WHOA WHOOPS");
		break;
	}

	return;
}

void color_display_debug() {
	struct RelativeColorType color_in_percent = color_read_percent();
	char buffer[DISPLAY_WIDTH];

	while (!menu_pressed && !ok_pressed) {
		color_in_percent = color_read_percent();
		display_clear();
		display_print_line("Color Readings:", strlen("Color Readings:"), 0);
		// display r, g, b percentages
		color_make_percent_line(buffer, color_in_percent.r_perc, 'r');
		display_print_line(buffer, DISPLAY_WIDTH, 1);
		color_make_percent_line(buffer, color_in_percent.g_perc, 'g');
		display_print_line(buffer, DISPLAY_WIDTH, 2);
		color_make_percent_line(buffer, color_in_percent.b_perc, 'b');
		display_print_line(buffer, DISPLAY_WIDTH, 3);

		HAL_Delay(1000);

	}

	reset_buttons();

}

struct RelativeColorType color_read_percent() {
	struct RelativeColorType color_in_percent;
	struct ColorType color_in = color_read_rgbc();

	color_in_percent = color_abs_to_rel(color_in);

	return color_in_percent;
}

struct RelativeColorType color_abs_to_rel(struct ColorType color_in) {
	struct RelativeColorType color_in_percent;
	int total_feedback = color_in.r + color_in.g + color_in.b;
	color_in_percent.r_perc = color_in.r / (float)total_feedback * 100.0;
	color_in_percent.g_perc = color_in.g / (float)total_feedback * 100.0;
	color_in_percent.b_perc = color_in.b / (float)total_feedback * 100.0;

	return color_in_percent;
}

struct ColorType color_read_rgbc() {
	struct ColorType color_in;
	color_in.r = color_read('r');
	color_in.g = color_read('g');
	color_in.b = color_read('b');
	color_in.c = color_read('c');

	return color_in;
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

void load_cell_init() {
	uint8_t buffer[3] = {0,0,0};
	HAL_StatusTypeDef ret;

	// first, check valid, and don't stop asking until we get a valid response

	buffer[0] = LOAD_CELL_PU_CTRL;
	buffer[1] = 0b00010110; // turn on analog and digital circuits

	// read status reg
	ret = HAL_I2C_Master_Transmit(&hi2c1, (LOAD_CELL_I2C_ADDR), buffer, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}
}

int load_cell_read() {
	uint8_t buffer[3] = {0,0,0};
	HAL_StatusTypeDef ret;

	// first, check valid, and don't stop asking until we get a valid response
	while (!buffer[0]) {
		buffer[0] = LOAD_CELL_PU_CTRL;

		// read status reg
		ret = HAL_I2C_Master_Transmit(&hi2c1, (LOAD_CELL_I2C_ADDR), buffer, 1, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
			while(1);
		}

		ret = HAL_I2C_Master_Receive(&hi2c1, (LOAD_CELL_I2C_ADDR), buffer, 1, HAL_MAX_DELAY);

		if ( ret != HAL_OK ) {
			while(1);
		}


		buffer[0] &= LOAD_CELL_DATA_READY_MASK; // check the 5th bit for adc valid
	}

	buffer[0] = LOAD_CELL_ADC_REG;

	ret = HAL_I2C_Master_Transmit(&hi2c1, (LOAD_CELL_I2C_ADDR), buffer, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, (LOAD_CELL_I2C_ADDR), buffer, 3, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		while(1);
	}

	int result = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	return result;
}

//void height_init() {
//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//}

float height_read_raw() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	float raw = (float)HAL_ADC_GetValue(&hadc1);
	return raw;
}

float height_read_cm() {
	float raw = height_read_raw();
	float ratio = 4873.099237; //TBD
	float height = (2560.0-560.0/(1-raw/ratio))/56.0;
    return height;
}

float extern height_read_cm_avg(int num_samples) {
	float average = 0;
	for (int i = 0; i < num_samples; i++) {
		average += height_read_cm() / (float)num_samples;
		HAL_Delay(3);
	}
	return average;
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

void display_init() {
	  display_on();
	  display_clear();
	  display_set_brightness(4);
	  display_set_contrast(50);
	  display_print_line("Initializing...", 15, 0);

	  char datestring[8];
	  char timestring[5];
	  RTC_TimeTypeDef currTime = {0};
	  RTC_DateTypeDef currDate = {0};
	  HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	  last_read_date = currDate;

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

// CODE TAKEN FROM (https://www.geeksforgeeks.org/find-number-of-days-between-two-given-dates/)
// did not have time to implement by myself

// To store number of days in
// all months from January to Dec.
const int monthDays[12]
    = { 31, 28, 31, 30, 31, 30,
       31, 31, 30, 31, 30, 31 };

// This function counts number of
// leap years before the given date
int countLeapYears(struct Date d)
{
    int years = d.y;

    // Check if the current year needs to be
    //  considered for the count of leap years
    // or not
    if (d.m <= 2)
        years--;

    // An year is a leap year if it
    // is a multiple of 4,
    // multiple of 400 and not a
     // multiple of 100.
    return years / 4
           - years / 100
           + years / 400;
}

// This function returns number of
// days between two given dates
int getDifference(struct Date dt1, struct Date dt2)
{
    // COUNT TOTAL NUMBER OF DAYS
    // BEFORE FIRST DATE 'dt1'

    // initialize count using years and day
    long int n1 = dt1.y * 365 + dt1.d;

    // Add days for months in given date
    for (int i = 0; i < dt1.m - 1; i++)
        n1 += monthDays[i];

    // Since every leap year is of 366 days,
    // Add a day for every leap year
    n1 += countLeapYears(dt1);

    // SIMILARLY, COUNT TOTAL NUMBER OF
    // DAYS BEFORE 'dt2'

    long int n2 = dt2.y * 365 + dt2.d;
    for (int i = 0; i < dt2.m - 1; i++)
        n2 += monthDays[i];
    n2 += countLeapYears(dt2);

    // return difference between two counts
    return (n2 - n1);
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
