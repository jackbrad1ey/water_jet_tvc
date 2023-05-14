#include "stm32f4xx_hal.h";

const float MIN_DUTY_CYCLE;
const float MAX_DUTY_CYCLE;
const float MAX_ANGLE;
const float MIN_ANGLE;

float _degrees_to_duty_cycle(float degrees);
float set_motor(int motor_id, float degrees, TIM_HandleTypeDef htim);