/**
  ******************************************************************************
  * @file           : hacheur.c
  * @brief          file to control the chopper of the DCM
  ******************************************************************************
  */

/* Includes --------------------------------------------------------------*/
#include "hacheur.h"

/* Variables ------------------------------------------------------------ */

extern const uint8_t power_on[];
extern const uint8_t power_off[];
extern const uint8_t not_found[];

/* Functions ------------------------------------------------------------ */

/**
 * @brief Turns the power ON of the chopper
 *
 * */
void start(void){
	uart_write((uint8_t*) power_on,10);
	HAL_GPIO_WritePin(Start_Command_GPIO_Port, Start_Command_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Start_Command_GPIO_Port, Start_Command_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Start_Command_GPIO_Port, Start_Command_Pin, GPIO_PIN_RESET);
	speed(50);
}

/**
 * @brief Function to control the speed of the MCC
 *
 * @param[in] alpha : int, value of the duty cycle wanted.
 *
 * */
void speed(int alpha){
	int compare_value;
	compare_value = alpha*(1024-1)/100;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare_value);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare_value);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}
