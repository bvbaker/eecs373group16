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

struct ColorType {
	uint16_t r, g, b, c;
};

struct RelativeColorType {
	float r_perc, g_perc, b_perc;
};

struct NutritionType {
	float caffeine_mg, sugar_g, sodium_mg, calories, carbs_g, protein_g, fat_g;
};

struct DrinkType {
	struct RelativeColorType max, min;
	float serving_size_ml;
	struct NutritionType nutrition_per_serving;
	char name[20];
	int already_guessed;
};

struct DayType {
	struct NutritionType nutrition_total;
//	RTC_DateTypeDef date;
//	int counted;
};

struct WeekType {
	struct DayType day[7];
};

// CODE TAKEN FROM (https://www.geeksforgeeks.org/find-number-of-days-between-two-given-dates/)
// did not have time to implement by myself
// A date has day 'd', month 'm' and year 'y'
struct Date {
    int d, m, y;
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
I2C_HandleTypeDef extern hi2c1;

ADC_HandleTypeDef extern hadc1;

RTC_HandleTypeDef extern hrtc;

struct WeekType extern this_week;

RTC_DateTypeDef extern last_read_date;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Time Functions
void extern week_add_measurement(struct DrinkType drink_in, float volume_ml);
void extern week_reset();
void extern week_reset_one_day(int index);

// CODE TAKEN FROM (https://www.geeksforgeeks.org/find-number-of-days-between-two-given-dates/)
// did not have time to implement by myself
// dt1 is older than dt2
int extern getDifference(struct Date dt1, struct Date dt2);

// Nutrition Functions
struct NutritionType extern nutrition_accumulate_amount(struct NutritionType accumulator, struct NutritionType addition, float volume_ml, float serving_size_ml);

struct DrinkType extern drink_get_empty();

// Color Functions
void extern color_init();
void extern color_off();
uint16_t extern color_read(char color);
struct ColorType extern color_read_rgbc();
struct RelativeColorType extern color_read_percent();
struct RelativeColorType extern color_abs_to_rel(struct ColorType color_in);
void extern color_display_debug();
void extern color_make_percent_line(char* buffer, float percent, char color);
struct RelativeColorType extern color_read_percent_average(int num_samples);

// Load Cell Functions
void extern load_cell_init();
int extern load_cell_read();

// Button Variables (0 when not pressed, 1 when pressed and not handled)
int extern up_pressed;
int extern down_pressed;
int extern menu_pressed;
int extern ok_pressed;

// Miscellaneous
void extern reset_buttons();
void extern dumb_way_to_update_week();

// Display Functions
void extern display_test();
void extern string_to_uint8_t(char* str, uint8_t* buff, int len);
void extern display_clear();
void extern display_print_line(char* str, int len, int line);
void extern display_set_cursor_line(int line);
void extern display_on();
void extern display_off();
void extern display_set_brightness(uint8_t brightness_in);
void extern display_init();
void extern display_set_contrast(uint8_t contrast_in);
int extern display_guess(struct DrinkType guessed_drink);
void extern display_day_summary(struct DayType day_in, RTC_DateTypeDef date_in);
void extern display_week_summary();
void extern display_time();
void extern display_level();

// Menu Functions
void extern menu_call();
void extern menu_init();
void extern menu_display(int menu_idx);
void extern menu_select(int menu_idx);

// Guess Functions
void extern guess_liquid();
int extern guess_within_range(struct DrinkType drink, struct RelativeColorType measured);

// Height Functions
//void extern height_init();
float extern height_read_raw();
float extern height_read_cm();
float extern height_read_cm_avg(int num_samples);
float extern volume_ml_read_avg();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
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
#define	DISPLAY_COLOR 1
#define CHECK_DAY 2
#define CHECK_WEEK 3
#define CHECK_LEVEL 4
#define DISPLAY_TIME 5
#define SET_DEMO_VALUES 6
#define RESET_DAY_OR_WEEK 7

// Height Defines
#define NUM_HEIGHT_SAMPLES (10)

// Volume Defines
#define PI (3.14159265359)
#define DIAMETER_CM (8.0)
#define RADIUS_CM (DIAMETER_CM / 2.0)
#define VOLUME_OFFSET_ML (2.0)

// Guess Defines
#define NUM_COLOR_SAMPLES (10)
#define NUM_DRINKS (2)
#define MTN_DEW_REGULAR (0)
#define MTN_DEW_CODE_RED (1)
#define DRINK_NONE (NUM_DRINKS)
#define DRINK_UNKNOWN (NUM_DRINKS + 1)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
