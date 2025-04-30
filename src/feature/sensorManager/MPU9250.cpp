#include "MPU9250.h"

static const char *TAG = "MPU9250";

// MPU9250.cpp
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;


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

void MPU9250::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // Normalize accelerometer
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // avoid division by zero
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated direction of gravity
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Optional integral feedback
    if (TWO_KI > 0.0f) {
        integralFBx += TWO_KI * halfex * dt;
        integralFBy += TWO_KI * halfey * dt;
        integralFBz += TWO_KI * halfez * dt;
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }

    // Apply proportional feedback
    gx += TWO_KP * halfex;
    gy += TWO_KP * halfey;
    gz += TWO_KP * halfez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float MPU9250::getRoll()
{
    return atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
}

float MPU9250::getPitch()
{
    return asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
}

void MPU9250::Task()
{
    esp_err_t ret = i2c_master_init();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized");

    initMPU9250();
    lastTime = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Initialisation des angles et du filtre
    float rollGyro = 0.0f;
    float pitchGyro = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;

    while (true) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0.0f) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        lastTime = now;

        // Lecture des capteurs
        float ax = readAccel(0);
        float ay = readAccel(2);
        float az = readAccel(4);

        float gx = readGyro(0);
        float gy = readGyro(2);
        float gz = readGyro(4);

        // Offsets
        ax -= OFFSET_AX;
        ay -= OFFSET_AY;
        az -= OFFSET_AZ;

        gx -= OFFSET_GX;
        gy -= OFFSET_GY;
        gz -= OFFSET_GZ;

        // Accéléromètre : calcul du roll/pitch
        // float rollAcc  = atan2f(ay, az) * RAD_TO_DEG;
        // float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

        // // Gyroscope : intégration
        // rollGyro  += gx * dt;
        // pitchGyro += gy * dt;

        // // Filtre complémentaire
        // roll  = rollGyro * alpha + (1.0f - alpha) * rollAcc ;
        // pitch = pitchGyro * alpha + (1.0f - alpha) * pitchAcc ;

        // Convert gyro from deg/s to rad/s
        gx *= (M_PI / 180.0f);
        gy *= (M_PI / 180.0f);
        gz *= (M_PI / 180.0f);

        // Update filter (assuming 100Hz update rate, so dt = 0.01)
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, 0.01f);

        // Get roll and pitch
        float roll  = getRoll();
        float pitch = getPitch();

        ESP_LOGI(TAG,"Roll: %.2f, Pitch: %.2f\n", roll, pitch);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
