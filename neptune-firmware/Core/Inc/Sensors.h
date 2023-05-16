/*
 * Sensors.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Angus McLennan
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
#include <stdint.h>
/*
 * Data structure to hold BMX055 data
 * Data elements array for each sensor:
 * Data X, Data Y, Data Z, Current sample time (uS), Previous sample time (uS)
 */
typedef struct {
	float accel[5];
	float gyro[5];
	float mag[5];
}BMX055_Data_Handle;

typedef struct {
	float pressure;
	float temperature;
	float altitude;
	uint32_t current_sample_time;
	uint32_t last_sample_time;
}MS5611_Data_Handle;

typedef struct {

}GPS_Data_Handle;

#endif /* INC_SENSORS_H_ */
