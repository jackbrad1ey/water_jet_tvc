/*
 * servos.c
 *
 *  Created on: 12 May 2023
 *      Author: jb
 */

#include "servos.h";
#include "usbd_cdc_if.h";
#include "string.h";

const float MIN_DUTY_CYCLE = 2.5;
const float MAX_DUTY_CYCLE = 10.5;
const float MAX_ANGLE = 180;
const float MIN_ANGLE = 0;  // may want to do -90 to 90 instead, we'll see how we go

float _degrees_to_duty_cycle(float degrees) {
	float duty_cycle = MIN_DUTY_CYCLE + ((MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) / (MAX_ANGLE - MIN_ANGLE)) * (degrees - MIN_ANGLE);

	return duty_cycle;
}

float set_motor(int motor_id, float degrees, TIM_HandleTypeDef htim) {
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

// we have our min, max, and need to normalise that to 0 to 180
