#ifndef MPU9250_H_
#define MPU9250_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define MPU9250_ADDR         0x68
#define MPU9250_WHO_AM_I_REG 0x75
#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H  0x43
#define MPU9250_TEMP_OUT_H   0x41

class MPU9250 {
public:
    MPU9250();

    esp_err_t init();

    esp_err_t getAccel(int16_t* ax, int16_t* ay, int16_t* az);

    esp_err_t getGyro(int16_t* gx, int16_t* gy, int16_t* gz);
    void Task();

private:
    i2c_port_t i2c_port_;
    int sda_pin_;
    int scl_pin_;
    uint32_t clock_speed_;

    void writeByte(uint8_t reg, uint8_t data);
    void readBytes(uint8_t reg, uint8_t *buf, uint8_t len);
    int16_t convert16Bit(uint8_t high, uint8_t low);
};

#endif