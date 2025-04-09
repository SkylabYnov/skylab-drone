#include "MPU9250.h"

static const char *TAG = "MPU9250";

MPU9250::MPU9250()
    : i2c_port_(I2C_NUM_0), sda_pin_(21), scl_pin_(22), clock_speed_(400000) {
    }

esp_err_t MPU9250::init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin_;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin_;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clock_speed_;
    conf.clk_flags = 0;

    esp_err_t res = i2c_param_config(i2c_port_, &conf);
    if (res != ESP_OK) return res;
    res = i2c_driver_install(i2c_port_, conf.mode, 0, 0, 0);
    if (res != ESP_OK) return res;

    writeByte(MPU9250_PWR_MGMT_1, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t who_am_i = 0;
    readBytes(MPU9250_WHO_AM_I_REG, &who_am_i, 1);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02x", who_am_i);

    return ESP_OK;
}

void MPU9250::writeByte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write byte error: %s", esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
}

void MPU9250::readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read bytes error: %s", esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
}

int16_t MPU9250::convert16Bit(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

esp_err_t MPU9250::getAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t data[6];
    readBytes(MPU9250_ACCEL_XOUT_H, data, 6);
    *ax = convert16Bit(data[0], data[1]);
    *ay = convert16Bit(data[2], data[3]);
    *az = convert16Bit(data[4], data[5]);
    return ESP_OK;
}

esp_err_t MPU9250::getGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[6];
    readBytes(MPU9250_GYRO_XOUT_H, data, 6);
    *gx = convert16Bit(data[0], data[1]);
    *gy = convert16Bit(data[2], data[3]);
    *gz = convert16Bit(data[4], data[5]);
    return ESP_OK;
}

void MPU9250::Task()
{
    if (init() != ESP_OK) {
        ESP_LOGE(TAG, "Erreur d'initialisation du MPU9250");
        return;
    }

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    while (true) {
        if (getAccel(&ax, &ay, &az) == ESP_OK) {
            ESP_LOGI(TAG, "Accel: X=%d Y=%d Z=%d", ax, ay, az);
        } else {
            ESP_LOGE(TAG, "Erreur de lecture de l'accéléromètre");
        }

        if (getGyro(&gx, &gy, &gz) == ESP_OK) {
            ESP_LOGI(TAG, "Gyro: X=%d Y=%d Z=%d", gx, gy, gz);
        } else {
            ESP_LOGE(TAG, "Erreur de lecture du gyroscope");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
