/*
 * servos.c
 *
 *  Created on: 12 May 2023
 *      Author: jb
 */

#include "servos.h"

float _degrees_to_duty_cycle(float degrees) {
	float duty_cycle = MIN_DUTY_CYCLE + ((MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) / (MAX_ANGLE - MIN_ANGLE)) * (degrees - MIN_ANGLE);

	return duty_cycle;
}

void set_motor(int motor_id, float degrees, TIM_HandleTypeDef htim) {
	float duty_cycle = _degrees_to_duty_cycle(degrees);

	// duty cycle = ccr / arr * 100
	float normalised = duty_cycle * 200 / 100;

	if (motor_id == 1) {
		htim.Instance->CCR1 = normalised;
	} else if (motor_id == 2) {
		htim.Instance->CCR2 = normalised;
	} else {
		// log an error over USB
		char buff[30];
		sprintf(buff, "ERROR: Invalid motor ID: %d\n", motor_id);
		CDC_Transmit_FS(buff, strlen(buff));
	}
}

void gimble_test(TIM_HandleTypeDef htim) {
	set_motor(1, 0, htim);
	HAL_Delay(1000);
	set_motor(1, 180, htim);
	HAL_Delay(1000);
	set_motor(1, 90, htim);
	HAL_Delay(1000);
	set_motor(2, 0, htim);
	HAL_Delay(1000);
	set_motor(2, 180, htim);
	HAL_Delay(1000);
	set_motor(2, 90, htim);
	HAL_Delay(1000);

	for (int angle=0; angle < 180; angle++) {
		set_motor(1, angle, htim);
		set_motor(2, angle, htim);
		HAL_Delay(10);
	}
}
// we have our min, max, and need to normalise that to 0 to 180
