#include "main.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include <stdint.h>
#include <math.h>

uint8_t imu_read8(SPI_HandleTypeDef hspi, char ce, uint8_t addr, uint8_t *data);
uint8_t imu_read_sub16(SPI_HandleTypeDef hspi, char ce, uint8_t addr, int shift, uint16_t *data);
uint8_t get_ang_rate(SPI_HandleTypeDef hspi, char axis, uint16_t *data);
uint8_t get_roll_and_pitch(SPI_HandleTypeDef hspi, float *roll, float *pitch);
