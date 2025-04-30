#include "MPU9250.h"

static const char *TAG = "MPU9250";

// MPU9250.cpp
float MPU9250::roll =0.0f;
float MPU9250::pitch =0.0f;


MPU9250::MPU9250() {
    }

esp_err_t MPU9250::i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed");
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed");
    }
    return err;
}

esp_err_t MPU9250::writeRegister(uint8_t devAddr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, devAddr, data, sizeof(data),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t MPU9250::readRegisters(uint8_t devAddr, uint8_t reg, uint8_t count, uint8_t *dest)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, devAddr, &reg, 1, dest, count,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

void MPU9250::initMPU9250()
{
    writeRegister(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00); // Wake up
    vTaskDelay(pdMS_TO_TICKS(100));
    // Enable magnetometer bypass to access AK8963 directly
    writeRegister(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);
    vTaskDelay(pdMS_TO_TICKS(10));
}

float MPU9250::readAccel(uint8_t axisOffset)
{
    uint8_t rawData[2] = {0};
    if (readRegisters(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H + axisOffset, 2, rawData) == ESP_OK) {
        int16_t value = (((int16_t)rawData[0]) << 8) | rawData[1];
        return (float)value / 16384.0f;
    }
    return 0.0f;
}

float MPU9250::readGyro(uint8_t axisOffset)
{
    uint8_t rawData[2] = {0};
    if (readRegisters(MPU9250_ADDR, MPU9250_GYRO_XOUT_H + axisOffset, 2, rawData) == ESP_OK) {
        int16_t value = (((int16_t)rawData[0]) << 8) | rawData[1];
        return (float)value / 131.0f;
    }
    return 0.0f;
}

void MPU9250::calibrate_gyro_offsets()
{
    const int samples = 5000;
    float gxSum = 0, gySum = 0, gzSum = 0;
    float axSum = 0, aySum = 0, azSum = 0;

    for (int i = 0; i < samples; i++) {
        gxSum += readGyro(0);
        gySum += readGyro(2);
        gzSum += readGyro(4);

        axSum += readAccel(0);
        aySum += readAccel(2);
        azSum += readAccel(4);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    OFFSET_GX = gxSum / samples;
    OFFSET_GY = gySum / samples;
    OFFSET_GZ = gzSum / samples;
    
    OFFSET_AX = axSum / samples;
    OFFSET_AY = aySum / samples;
    OFFSET_AZ = azSum / samples;

    ESP_LOGI(TAG, "Gyro offsets: GX=%.3f, GY=%.3f, GZ=%.3f, AX=%.3f, AY=%.3f, AZ=%.3f", OFFSET_GX, OFFSET_GY, OFFSET_GZ, OFFSET_AX, OFFSET_AY, OFFSET_AZ);
}

void MPU9250::Task()
{
    float rollGyro = 0.0f;
    float pitchGyro = 0.0f;
    

    esp_err_t ret = i2c_master_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized");

    uint32_t lastTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    initMPU9250();
    lastTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    initMPU9250();
    calibrate_gyro_offsets();

    while (true) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0.0f) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastTime = now;

        // Read accelerometer and gyroscope
        float ax = readAccel(0) - OFFSET_AX;
        float ay = readAccel(2) - OFFSET_AY;
        float az = readAccel(4) - OFFSET_AZ;

        float gx = readGyro(0) - OFFSET_GX;
        float gy = readGyro(2) - OFFSET_GY;
        float gz = readGyro(4) - OFFSET_GZ;

        // Accelerometer: calculate roll/pitch
        float rollAcc  = atan2f(ay, az) * 180.0f / M_PI;
        float pitchAcc = atanf(-ax / sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

        // Integrate gyro data
        rollGyro  += gx * dt;
        pitchGyro += gy * dt;

        // Complementary filter
        MPU9250::roll  = alphaRoll * rollGyro + (1.0f - alphaRoll) * rollAcc;
        MPU9250::pitch = alphaPitch * pitchGyro + (1.0f - alphaPitch) * pitchAcc;


        ESP_LOGI(TAG, "Roll: %7.2f | Pitch: %7.2f || RollAcc: %7.2f | PitchAcc: %7.2f | RollGyro: %7.2f | PitchGyro: %7.2f",
            MPU9250::roll, MPU9250::pitch, rollAcc, pitchAcc, rollGyro, pitchGyro);

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}
