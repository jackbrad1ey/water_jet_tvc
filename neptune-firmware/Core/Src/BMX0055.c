#include "BMX0055.h";

// GYRO
// bit 0 = read/write, 1 if read
// bits 1-7 = address
// bits 8-15 = data written to/from address

uint8_t gyro_read8(SPI_HandleTypeDef hspi, uint8_t addr, uint8_t *data) {
    // we're using full-duplex so we need to read as we write
    // and vice versa, hence the dummy data
    uint8_t txBuff[2] = {(addr | 0x80), 0x00};
    uint8_t rxBuff[2];

    HAL_GPIO_WritePin(GPIOA, GYR_CE_GPIO_Port, GPIO_PIN_RESET);
    uint8_t status = (HAL_SPI_TransmitReceive(&hspi, txBuff, rxBuff, 2, HAL_MAX_DELAY) == HAL_OK);
    HAL_GPIO_WritePin(GPIOA, GYR_CE_GPIO_Port, GPIO_PIN_SET);

    *data = rxBuff[1];  // first byte is dummy, second is real data

    return status;
}

uint16_t * get_axis_angle(SPI_HandleTypeDef hspi, char axis, uint16_t prev_angle, float prev_time) {
    uint8_t rate_lsb;
    uint8_t rate_msb;

    if (axis == 'x') {
        gyro_read8(hspi, 0x02, &rate_lsb);
        gyro_read8(hspi, 0x03, &rate_msb);
    } else if (axis == 'y') {
        gyro_read8(hspi, 0x04, &rate_lsb);
        gyro_read8(hspi, 0x05, &rate_msb);
    } else if (axis == 'z') {
        gyro_read8(hspi, 0x06, &rate_lsb);
        gyro_read8(hspi, 0x07, &rate_msb);
    }

    uint16_t rate = (rate_msb << 8) | rate_lsb;  // merge 
}