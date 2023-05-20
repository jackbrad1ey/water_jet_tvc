#ifndef SERVOS_H
#define SERVOS_H

#include "stm32f4xx_hal.h"
#include "string.h"
#include "usbd_cdc_if.h"

#define MIN_DUTY_CYCLE 2.5
#define MAX_DUTY_CYCLE 10.5
#define MAX_ANGLE      180
#define MIN_ANGLE      0  // may want to do -90 to 90 instead, we'll see how we go

float _degrees_to_duty_cycle(float degrees);
float _byte_to_duty_cycle(float byte);
void set_motor(int motor_id, int mode, float input, TIM_HandleTypeDef htim);
void gimble_test(TIM_HandleTypeDef htim);

typedef struct {
    float m1_angle;
    float m2_angle;
} Motors;

#endif
