#ifndef MPU9250_H_
#define MPU9250_H_

#include <features/sensorManager/Orientation.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include <math.h>

// I2C master configuration
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21 // Adjust as needed
#define I2C_MASTER_SCL_IO 22 // Adjust as needed
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_TIMEOUT_MS 1000

// MPU9250 registers and addresses
#define MPU9250_ADDR 0x68
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_INT_PIN_CFG 0x37

// AK8963 (magnetometer) definitions
#define AK8963_ADDR 0x0C
#define AK8963_CNTL1 0x0A // Control register 1 (for setting continuous mode)

#define RAD_TO_DEG 57.295779513

class MPU9250
{
public:
    float OFFSET_AX = 0.00f;
    float OFFSET_AY = 0.00f;
    float OFFSET_AZ = 0.00f;
    float OFFSET_GX = 0.00f;
    float OFFSET_GY = 0.00f;
    float OFFSET_GZ = 0.00f;

    float alphaRoll = 0.97f;  // Constante du filtre complémentaire
    float alphaPitch = 0.96f; // Constante du filtre complémentaire

    static SemaphoreHandle_t xOrientationMutex;
    static Orientation orientation;

    MPU9250();

    void Task();

private:
    uint32_t lastTime = 0;
    esp_err_t i2c_master_init();
    esp_err_t writeRegister(uint8_t devAddr, uint8_t reg, uint8_t value);
    esp_err_t readRegisters(uint8_t devAddr, uint8_t reg, uint8_t count, uint8_t *dest);
    void initMPU9250();
    float readAccel(uint8_t axisOffset);
    float readGyro(uint8_t axisOffset);

    void calibrate_gyro_offsets();
};

#endif