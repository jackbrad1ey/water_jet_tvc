#include "BMX0055.h";

// GYRO
// bit 0 = read/write, 1 if read
// bits 1-7 = address
// bits 8-15 = data written to/from address

uint8_t imu_read8(SPI_HandleTypeDef hspi, char ce, uint8_t addr, uint8_t *data) {
    uint16_t chip_enable;
    GPIO_TypeDef *port;

    if (ce == 'g') {
        chip_enable = GYR_CE_Pin;
        port = GYR_CE_GPIO_Port;
    } else if (ce == 'a') {
        chip_enable = ACC_CE_Pin;
        port = ACC_CE_GPIO_Port;
    } else if (ce == 'm') {
        chip_enable = MAG_CE_Pin;
        port = MAG_CE_GPIO_Port;
    } else {
        char buff[25];
        sprintf("ERROR: Invalid CE: %c\n", ce);
        CDC_Transmit_FS(buff, strlen(buff));
    }
    // we're using full-duplex so we need to read as we write
    // and vice versa, hence the dummy data
    uint8_t txBuff[2] = {(addr | 0x80), 0x00};
    uint8_t rxBuff[2];

    HAL_GPIO_WritePin(port, chip_enable, GPIO_PIN_RESET);
    uint8_t status = (HAL_SPI_TransmitReceive(&hspi, txBuff, rxBuff, 2, HAL_MAX_DELAY) == HAL_OK);
    HAL_GPIO_WritePin(port, chip_enable, GPIO_PIN_SET);

    *data = rxBuff[1];  // first byte is dummy, second is real data

    return status;
}

uint8_t imu_read_sub16(SPI_HandleTypeDef hspi, char ce, uint8_t addr, int shift, uint16_t *data) {
    uint16_t chip_enable;
    GPIO_TypeDef *port;

    if (ce == 'g') {
        chip_enable = GYR_CE_Pin;
        port = GYR_CE_GPIO_Port;
    } else if (ce == 'a') {
        chip_enable = ACC_CE_Pin;
        port = ACC_CE_GPIO_Port;
    } else if (ce == 'm') {
        chip_enable = MAG_CE_Pin;
        port = MAG_CE_GPIO_Port;
    } else {
        char buff[25];
        sprintf("ERROR: Invalid CE: %c\n", ce);
        CDC_Transmit_FS(buff, strlen(buff));
    }

    uint8_t txBuff[3] = {(addr | 0x80), 0x00, 0x00};
    uint8_t rxBuff[3];

    HAL_GPIO_WritePin(port, chip_enable, GPIO_PIN_RESET);
    uint8_t status = (HAL_SPI_TransmitReceive(&hspi, txBuff, rxBuff, 3, HAL_MAX_DELAY) == HAL_OK);
    HAL_GPIO_WritePin(port, chip_enable, GPIO_PIN_SET);

    *data = rxBuff[1] | (rxBuff[2] << shift);  // first byte is dummy, second two are real

    return status;
}

uint8_t get_ang_rate(SPI_HandleTypeDef hspi, char axis, uint16_t *data) {
    uint16_t rate;
    uint8_t res;

    if (axis == 'x') {
        res = imu_read_sub16(hspi, 'g', 0x02, 8, &rate);
    } else if (axis == 'y') {
        res = imu_read_sub16(hspi, 'g', 0x04, 8, &rate);
    } else if (axis == 'z') {
        res = imu_read_sub16(hspi, 'g', 0x06, 8, &rate);
    } else {
        char buff[30];
        sprintf("ERROR: Invalid axis: %c\n", axis);
        CDC_Transmit_FS(buff, strlen(buff));
    }

    if (res) {
        return 1;
    } else {
        *data = rate;
        return 0;
    }
}

uint8_t get_roll_and_pitch(SPI_HandleTypeDef hspi, float *roll, float *pitch) {
    // unless changed, we're at +-2g, with 16 bits worth of info between those ranges
    // 0.98 mg/LSB
    uint16_t raw_x;
    uint16_t raw_y;
    uint16_t raw_z;

    if (imu_read_sub16(hspi, 'a', 0x02, 4, &raw_x)) {
        return 1;
    } else if (imu_read_sub16(hspi, 'a', 0x04, 4, &raw_y)) {
        return 1;
    } else if (imu_read_sub16(hspi, 'a', 0x06, 4, &raw_z)) {
        return 1;
    }

    float xg = (raw_x * 0.98) / 1000 - 2;
    float yg = (raw_y * 0.98) / 1000 - 2;
    float zg = (raw_z * 0.98) / 1000 - 2;

    *roll = atan2(yg, zg) * 57.3;
    *pitch = atan2(-xg, sqrt(yg*yg + zg*zg)) * 57.3;

    return 0;
}