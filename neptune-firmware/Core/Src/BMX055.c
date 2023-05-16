#include "BMX055.h"

/*!
 * @brief This internal API is used to perform advanced self test for Z axis
 *
 * @param[in] bmx055     : Structure instance of bmx055
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 *
 * Return value         | Status of self-test
 *----------------------|---------------------------
 *      0               | BMX055_MAG_ADV_SELF_TEST_SUCCESS
 *      8               | BMX055_MAG_ADV_SELF_TEST_FAIL
 */
static int8_t perform_adv_self_test(BMX055_Handle *bmx055);

/*!
 * @brief This internal API is used to perform the normal self test
 * of the sensor and return the self test result as return value
 *
 * @param[in] bmx055          : Structure instance of bmx055.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
static int8_t perform_normal_self_test(BMX055_Handle *bmx055);
/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
 */
bool BMX055_init(BMX055_Handle *bmx055) {
	// Set CS pins HIGH
	HAL_GPIO_WritePin(bmx055->acc_CS_port, bmx055->acc_CS_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(bmx055->gyro_CS_port, bmx055->gyro_CS_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(bmx055->mag_CS_port, bmx055->mag_CS_pin, GPIO_PIN_SET);

	// Set accel scale factor and map to m/s^2
	// 12bit (4096) accelerometer maps to specified range. This is used to calculate scale factor.
	// Note: +-16g is range of 32g
	switch (bmx055->acc_range) {
	case BMX055_ACC_RANGE_16:
		// 16g
		bmx055->acc_rescale = (2.0 * 16.0 / 4096.0) * 9.81;
		break;
	case BMX055_ACC_RANGE_8:
		// 8g
		bmx055->acc_rescale = (2.0 * 8.0 / 4096.0) * 9.81;
		break;
	case BMX055_ACC_RANGE_4:
		// 4g
		bmx055->acc_rescale = (2.0 * 4.0 / 4096.0) * 9.81;
		break;
	case BMX055_ACC_RANGE_2:
		// 2g
		bmx055->acc_rescale = (2.0 * 2.0 / 4096.0) * 9.81;
		break;
	}

	// Set gyro scale factor for map to degrees and map from deg to rad
	// 16bit (65536) gyro maps to specified range. This is used to calculate scale factor.
	// Note: +-2000 deg/s is range of 4000 deg/s
	switch (bmx055->gyro_range) {
	case BMX055_GYRO_RANGE_16_4:
		// 2000
		bmx055->gyro_rescale = (2.0 * 2000.0 / 65536.0) * (M_PI / 180.0);
		break;
	case BMX055_GYRO_RANGE_32_8:
		// 1000
		bmx055->gyro_rescale = (2.0 * 1000.0 / 65536.0) * (M_PI / 180.0);
		break;
	case BMX055_GYRO_RANGE_65_6:
		// 500
		bmx055->gyro_rescale = (2.0 * 500.0 / 65536.0) * (M_PI / 180.0);
		break;
	case BMX055_GYRO_RANGE_131_2:
		// 250
		bmx055->gyro_rescale = (2.0 * 250.0 / 65536.0) * (M_PI / 180.0);
		break;
	case BMX055_GYRO_RANGE_262_4:
		// 125
		bmx055->gyro_rescale = (2.0 * 125.0 / 65536.0) * (M_PI / 180.0);
		break;
	}

	// Set mag scale factor to units of uT
	// XY axes are 12bit (4096) and Z axis is 15bit (32768).
	// XY axis maximum value is +-1300uT and Z axis maximum value is +-2500uT
	bmx055->mag_rescale_xy = (2.0*1300.0 / 4096.0);
	bmx055->mag_rescale_z = (2.0*2500.0 / 32768.0);


	if (BMX055_searchDevice(bmx055)) {
		BMX055_configuration(bmx055);
//		uint8_t res = perform_adv_self_test(bmx055);
		if (!perform_normal_self_test(bmx055))
			return true;
		else
			return false;
	} else {
		return false;
	}
}

/**
 * @brief Search bmx055
 * @retval true  Found Device
 * @retval false : Not Found Device
 */
bool BMX055_searchDevice(BMX055_Handle *bmx055) {
	uint8_t acc_device = 0x00;
	uint8_t mag_device = 0x00;

	// Mag SoftReset
	uint8_t data = 0x82;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_POW_CTL_REG, &data, 1);
	HAL_Delay(2);

	/* Mag Setting */
	// set sleep mode
	data = BMX055_MAG_POW_CTL_SLEEP_MODE;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_POW_CTL_REG, &data, 1);
	HAL_Delay(3);

	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_WHO_AM_I_REG, &acc_device, 1);

	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_REP_CHIP_ID, &mag_device, 1);

	if (acc_device == BMX055_ACC_DEVICE && mag_device == BMX055_MAG_DEVICE) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Set Config
 */
void BMX055_configuration(BMX055_Handle *bmx055) {
	/* SoftReset */
	uint8_t data = BMX055_INITIATED_SOFT_RESET;
	// Accel SoftReset
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_RESET_REG, &data, 1);
	HAL_Delay(2);  // wait 1.8ms
	// Gyro SoftReset
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_RESET_REG, &data, 1);
	HAL_Delay(2);  // wait 1.8ms

	// adv.st, DataRate, OperationMode, SelfTest (NomalMode)
	data = bmx055->mag_data_rate;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);
	// Repetitions for X-Y Axis  0x04 -> 0b00000100 -> (1+2(2^2)) = 9
	data = 0x04;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_REP_XY_REG, &data, 1);
	// Repetitions for Z-Axis  0x0F-> 0b00001111-> (1 +(2^0 + 2^1 + 2^2 + 2^3) = 15
	data = 0x0F;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_REP_Z_REG, &data, 1);

	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);

	/* Accel Setting */
	// Select Accel PMU Range
	data = bmx055->acc_range;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_RANGE_REG, &data, 1);
	// Select Accel PMU_BW
	data = bmx055->acc_range;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_BW_REG, &data, 1);
	// Select Accel PMU_LPW  (NomalMode, SleepDuration 0.5ms)
	data = BMX055_ACC_PMU_LPW_MODE_NOMAL | BMX055_ACC_PMU_LPW_SLEEP_DUR_0_5MS;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_LPW_REG, &data, 1);

	/* Gyro Setting */
	// Select Gyro Range
	data = bmx055->gyro_range;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_RANGE_REG, &data, 1);
	// Select Gyro BW
	data = bmx055->gyro_bandwidth;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_BW_REG, &data, 1);
	// Select Gyro LPM1 (NomalMode, SleepDuration 2ms)
	data = BMX055_GYRO_LPM1_MODE_NOMAL | BMX055_GYRO_LPM1_SLEEP_DUR_2MS;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_LPM1_REG, &data, 1);

	HAL_Delay(200);
}

/**
 * @brief Read Accel
 * @param [out] *accl : accel value  (X-accel : accl[0], Y-accel : accl[1], Z-accel : accl[2])
 */
void BMX055_readAccel(BMX055_Handle *bmx055, float *accl) {
	uint16_t accl_data[6] = { 0 };
	int accel_read[3];

	// read accel value
	for (int i = 0; i < 6; i++) {
		BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin,
		BMX055_ACC_DATA_START_REG + i, &accl_data[i], 1);
	}

	// conv data  accel:12bit
	accel_read[0] = ((accl_data[1] << 4) | (accl_data[0] >> 4));

	if (accel_read[0] > 2047) {
		accel_read[0] -= 4096;
	}
	accl[0] = accel_read[0] * bmx055->acc_rescale;

	accel_read[1] = ((accl_data[3] << 4) | (accl_data[2] >> 4));
	if (accel_read[1] > 2047) {
		accel_read[1] -= 4096;
	}
	accl[1] = accel_read[1] * bmx055->acc_rescale;

	accel_read[2] = ((accl_data[5] << 4) | (accl_data[4] >> 4));
	if (accel_read[2] > 2047) {
		accel_read[2] -= 4096;
	}
	accl[2] = accel_read[2] * bmx055->acc_rescale;
}

/**
 * @brief Read Gyro
 * @param [out] *gyro gyro value (X-gyro: gyro[0], Y-gyro: gyro[1], Z-gyro: gyro[2])
 */
void BMX055_readGyro(BMX055_Handle *bmx055, float *gyro) {
	uint8_t gyro_data[6];
	int gyro_read[3];

	// read gyro value
	for (int i = 0; i < 6; i++) {
		BMX055_readSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin,
		BMX055_GYRO_DATA_START_REG + i, &gyro_data[i], 1);
	}

	// conv data  gyro:16bit
	gyro_read[0] = ((gyro_data[1] << 8) + gyro_data[0]);
	if (gyro_read[0] > 32767) {
		gyro_read[0] -= 65536;
	}
	gyro[0] = gyro_read[0] * bmx055->gyro_rescale;

	gyro_read[1] = ((gyro_data[3] << 8) + gyro_data[2]);
	if (gyro_read[1] > 32767) {
		gyro_read[1] -= 65536;
	}
	gyro[1] = gyro_read[1] * bmx055->gyro_rescale;

	gyro_read[2] = ((gyro_data[5] << 8) + gyro_data[4]);
	if (gyro_read[2] > 32767) {
		gyro_read[2] -= 65536;
	}
	gyro[2] = gyro_read[2] * bmx055->gyro_rescale;
}

/**
 * @brief Read Mag
 * @param [out] *mag mag value (X-mag: mag[0], Y-mag: mag[1], Z-mag: mag[2])
 */
void BMX055_readRawMag(BMX055_Handle *bmx055, float *mag) {
	uint8_t mag_data[8];

	// read mag value
	for (int i = 0; i < 8; i++) {
		BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin,
		BMX055_MAG_DATA_START_REG + i, &mag_data[i], 1);
	}

	// conv data  mag x:12bit
	mag[0] = ((int16_t) (mag_data[1] << 5) + (int16_t) (mag_data[0] >> 3));
	if (mag[0] > 4095) {
		mag[0] -= 8192;
	}

	// conv data  mag y:12bit
	mag[1] = ((int16_t) (mag_data[3] << 5) + (int16_t) (mag_data[2] >> 3));
	if (mag[1] > 4095) {
		mag[1] -= 8192;
	}

	// conv data  mag z:15bit
	mag[2] = ((int16_t) (mag_data[5] << 7) + (int16_t) (mag_data[4] >> 1));
	if (mag[2] > 16383) {
		mag[2] -= 32768;
	}
}

arm_status BMX055_readCompensatedMag(BMX055_Handle *bmx055, float *mag) {
	// Read raw mag data
	BMX055_readRawMag(bmx055, mag);

	// Apply scale factor to raw mag data
	mag[0] *= bmx055->mag_rescale_xy;
	mag[1] *= bmx055->mag_rescale_xy;
	mag[2] *= bmx055->mag_rescale_z;

	// Put data into dsp struct
	arm_matrix_instance_f32 raw_data;
	arm_mat_init_f32(&raw_data, 3, 1, mag);

	// Apply hard iron compensation
	arm_matrix_instance_f32 hard_iron_compensated;
	float hard_iron_compensated_buff[3];
	arm_mat_init_f32(&hard_iron_compensated, 3, 1, hard_iron_compensated_buff);

	arm_status result = arm_mat_sub_f32(&raw_data, &bmx055->mag_hard_iron_offsets, &hard_iron_compensated);
	if (result)
		return result;

	// Apply soft iron compensation
	arm_matrix_instance_f32 soft_iron_compensated;
	float soft_iron_compensated_buff[3];
	arm_mat_init_f32(&soft_iron_compensated, 3, 1, soft_iron_compensated_buff);
	result = arm_mat_mult_f32(&bmx055->mag_soft_iron_offsets, &hard_iron_compensated, &soft_iron_compensated);

	// Return compensated data in uT
	memcpy(mag, &soft_iron_compensated.pData[0], 3 * sizeof(float));

	return result;
}

void BMX055_setInterrupts(BMX055_Handle *bmx055) {
	/* Disable interrupts, change configuration, wait 10ms, re-enable interrupts */
	uint8_t data;
	uint8_t read_data;

	// Disable INT1
	data = 0;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_INT_1_EN, &data, 1);
	HAL_Delay(10);
	// Map data ready interrupt to int 1
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_INT_1_MAP, &read_data, 1);
	read_data |= 1;
	// Set int1_data bit
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_INT_1_MAP, &read_data, 1);
	HAL_Delay(10);
	// Set INT1 to active low
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_INT_ACTIVE_LEVEL, &read_data, 1);
	// Reset int1_lvl bit
	read_data &= 0b11111110;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_INT_ACTIVE_LEVEL, &read_data, 1);
	HAL_Delay(10);
	// Set INT1 to data accelerometer data ready interrupt
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_DATA_READY_INT_EN, &read_data, 1);
	// Set data_en bit
	read_data |= 0b00010000;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_DATA_READY_INT_EN, &read_data, 1);
	HAL_Delay(10);
	// Remove interrupt latch
	data = 0b10000000;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_INT_RST_LATCH, &data, 1);
	HAL_Delay(10);

	// Disable INT3
	data = 0;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_INT_3_EN, &data, 1);
	HAL_Delay(10);
	// Map data ready interrupt to int 1
	read_data = 1;
	// Set int1_data bit
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_INT_1_MAP, &read_data, 1);
	HAL_Delay(10);
	// Set INT3 to active low
	data = 0;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_INT_ACTIVE_LEVEL, &data, 1);
	HAL_Delay(10);
	// Set INT3 to data gyroscope data ready interrupt
	data = 0b10000000;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_DATA_READY_INT_EN, &data, 1);
	HAL_Delay(100);
	// Remove interrupt latch
	data = 0b10000000;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_INT_RST_LATCH, &data, 1);
	HAL_Delay(10);

	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_AXES_REG, &read_data, 1);
	// Set Data Ready En, xyz axes, bit and reset DR Polarity, int en, latch bit
	data = (read_data | 0b10000000) & 0b10111000;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_AXES_REG, &data, 1);
	HAL_Delay(10);
}

void BMX055_exp_filter(float *prev_data, float *current_data, float *result, size_t len, float alpha) {
	arm_matrix_instance_f32 cur_dat;
	arm_matrix_instance_f32 prev_dat;
	arm_matrix_instance_f32 res;
	float current_data_float[3];
	float prev_data_float[3];
	arm_scale_f32((float*) current_data, alpha, (float*) current_data, len);
	arm_scale_f32((float*) prev_data, (1 - alpha), (float*) prev_data, len);
	// Copy and cast data into arrays. arm_mat_init requires float_32 arrays
	for (int i = 0; i < len; i++) {
		current_data_float[i] = (float) current_data[i];
		prev_data_float[i] = (float) prev_data[i];
	}
	arm_mat_init_f32(&cur_dat, 3, 1, current_data_float);
	arm_mat_init_f32(&prev_dat, 3, 1, prev_data_float);
	arm_mat_init_f32(&res, 3, 1, result);
	arm_mat_add_f32(&cur_dat, &prev_dat, &res);
}

/**
 * @brief Write SPI Data
 * @param [in] CS_Port
 * @param [in] CS_Pin
 * @param [in] register_addr
 * @param [in] data
 * @param [in] len
 */
void BMX055_writeSPI(BMX055_Handle *bmx055, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, uint8_t register_addr, uint8_t *data, size_t len) {
	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(bmx055->hspi, &register_addr, 1, 1000);
	HAL_SPI_Transmit(bmx055->hspi, data, len, 1000);
	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Read SPI Data
 * @param [in] device Device type (gyro, accel or mag)
 * @param [in] register_addr Register Address
 * @param [in] num Data Length
 * @param [out] *buf Read Data
 */
void BMX055_readSPI(BMX055_Handle *bmx055, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, uint8_t register_addr, uint8_t *data, size_t len) {
	// Add RW bit to start of register
	register_addr = register_addr | 0x80;
	uint8_t packet[20];

	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmx055->hspi, &register_addr, packet, len + 1, 1000);
	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

	// Copy data into "data" spot in memory
	memcpy(data, &packet[1], len);
}

/*!
 * @brief This internal API is used to perform advanced self test for Z axis
 */
static int8_t perform_adv_self_test(BMX055_Handle *bmx055) {
	int8_t rslt;
	int16_t positive_data_z;
	int16_t negative_data_z;
	uint8_t data;
	uint8_t read_data;

	/* Set the desired power mode ,axes control and repetition settings */
	// Set sleep mode
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &read_data, 1);
	data = read_data | 0b00000110; // Set opMode to sleep
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);

	// Disable x,y axis
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_AXES_REG, &read_data, 1);
	// Set Data Ready En, zyx axes, bit and reset DR Polarity, int en, latch bit
	data = (read_data | 0b10011000) & 0b10111000;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_AXES_REG, &data, 1);
	HAL_Delay(10);

	// Set z repetitions
	// Repetitions for Z-Axis  0x0F-> 0b00001111-> (1 +(2^0 + 2^1 + 2^2 + 2^3) = 15
	data = 0x0F;
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_REP_Z_REG, &data, 1);

	/* Measure the Z axes data with positive self-test current */
	// Set positive self test current
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &read_data, 1);
	data = read_data | 0b11000000; // Set Adv ST bits to 11
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);

	/* Set the device in forced mode */
	data = (data | 0b11000010) & 0b11111011; // Set device to forced mode -> Opmode should be 01
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);
	HAL_Delay(4);

	/* Read Mag data and store the value of Z axis data */
	int mag[4];
	BMX055_readRawMag(bmx055, mag);
	positive_data_z = (uint16_t) mag[2];

	/* Measure the Z axes data with negative self-test current */
	// Set negative self test current
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &read_data, 1);
	data = (read_data | 0b10000000) & 0b10111111; // Set Adv ST bits to 10
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);

	/* Set the device in forced mode */
	data = (data | 0b10000010) & 0b10111011; // Set device to forced mode -> Opmode should be 01
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);
	HAL_Delay(4);

	/* Read Mag data and store the value of Z axis data */
	BMX055_readRawMag(bmx055, mag);
	negative_data_z = (uint16_t) mag[2];

	/* Disable self-test current */
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &read_data, 1);
	data = read_data & 0b00111111; // Set Adv ST bits to 00
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);

	/* Validate the advanced self test */
	int32_t adv_self_test_rslt;

	/* Advanced self test difference between the Z axis mag data
	 * obtained by the positive and negative self-test current
	 */
	adv_self_test_rslt = positive_data_z - negative_data_z;
	/* Advanced self test validation */
	/*Value of adv_self_test_rslt should be in between 180-240 micro-tesla */
	if ((adv_self_test_rslt > 180) && (adv_self_test_rslt < 240)) {
		/* Advanced self test success */
		rslt = BMX055_MAG_ADV_SELF_TEST_SUCCESS;
	} else {
		/* Advanced self test fail */
		rslt = BMX055_MAG_ADV_SELF_TEST_FAIL;
	}

	return rslt;

}

/*!
 * @brief This internal API is used to perform normal self test
 */
static int8_t perform_normal_self_test(BMX055_Handle *bmx055) {
	uint8_t read_data;
	uint8_t data;
	// Set sleep mode
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &read_data, 1);
	data = read_data | 0b00000110; // Set opMode to sleep
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);

	/* Set the Self Test bit(bit0) of the 0x4C register */
	data = read_data | 0b00000001; // Set opMode to sleep
	BMX055_writeSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_ADV_OP_OUTPUT_REG, &data, 1);
	HAL_Delay(2);

	/* Validate normal self test */
	uint8_t self_test_rslt[3];
	uint8_t status;
	// Read the data from register 0x42, 0x44 and 0x46
	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_DATA_START_REG, &self_test_rslt[0], 1);

	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_DATA_START_REG + 2, &self_test_rslt[1], 1);

	BMX055_readSPI(bmx055, bmx055->mag_CS_port, bmx055->mag_CS_pin, BMX055_MAG_DATA_START_REG + 4, &self_test_rslt[2], 1);

	/* Combine the self test status and store it in the first
	 * 3 bits of the status variable for processing
	 */
	status = (uint8_t) ((self_test_rslt[2] & 1) & (self_test_rslt[1] & 1) & (self_test_rslt[0] & 1));

	if (status)
		return BMX055_MAG_SELF_TEST_SUCCESS;
	else
		return BMX055_MAG_SELF_TEST_FAIL;
}

void BMX055_accelCalibration(BMX055_Handle *bmx055, uint8_t verticalAxis) {
	// Set accelerometer into 2g mode
	uint8_t data = BMX055_ACC_RANGE_2;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_PMU_RANGE_REG, &data, 1);

	// Set offset compensation for axis that aligns with gravity vector
	switch (verticalAxis) {
	case BMX055_xAxis:
		data = 0b00000010;
		break;
	case BMX055_yAxis:
		data = 0b00001000;
		break;
	case BMX055_zAxis:
		data = 0b00100000;
		break;
	}
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB_TARGET, &data, 1);

	// Begin calibration
	// x axis
	data = 0b00100000;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	osDelay(100);
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	// Wait until cal_rdy flag is set
	while (!(data & 0b00010000) >> 4) {
		osDelay(100);
		BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	}

	// y axis
	data = 0b01010000;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin,
	BMX055_ACC_CALIB, &data, 1);
	osDelay(100);
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	// Wait until cal_rdy flag is set
	while (!(data & 0b00010000) >> 4) {
		osDelay(100);
		BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	}

	// z axis
	data = 0b01110000;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin,
	BMX055_ACC_CALIB, &data, 1);
	osDelay(100);
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	// Wait until cal_rdy flag is set
	while (!(data & 0b00010000) >> 4) {
		osDelay(100);
		BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, BMX055_ACC_CALIB, &data, 1);
	}

	// Set accelerometer back into previous mode
	data = bmx055->acc_range;
	BMX055_writeSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin,
	BMX055_ACC_PMU_RANGE_REG, &data, 1);

	// Read offsets for each axis
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, 0x3A, &data, 1);
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, 0x39, &data, 1);
	BMX055_readSPI(bmx055, bmx055->acc_CS_port, bmx055->acc_CS_pin, 0x38, &data, 1);
}

void BMX055_gyroCalibration(BMX055_Handle *bmx055) {
	// Set gyro range to 125deg/s mode
	uint8_t data = BMX055_GYRO_RANGE_262_4;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_RANGE_REG, &data, 1);

	// Set calibration to use filtered data
	BMX055_readSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_OFFSET_COMP, &data, 1);
	data &= 0x7F;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_OFFSET_COMP, &data, 1);

	// Set number of samples to 256 and begin calibration
	data = 0b00111111;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_A_FOC, &data, 1);

	// Wait for calibration to finish
	osDelay(100);
	BMX055_readSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_A_FOC, &data, 1);
	// Wait until fast_offset_en is reset
	while ((data & 0b00001000) >> 3) {
		osDelay(100);
		BMX055_readSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_A_FOC, &data, 1);
	}
	// Set gyro back to previous range
	data = bmx055->gyro_range;
	BMX055_writeSPI(bmx055, bmx055->gyro_CS_port, bmx055->gyro_CS_pin, BMX055_GYRO_RANGE_REG, &data, 1);
}
