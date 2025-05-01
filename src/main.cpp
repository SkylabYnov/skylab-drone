#include "feature/gpioManager/gpioManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "feature/espNowHandler/EspNowHandler.h"
#include "feature/motorManager/MotorManager.h"
#include "feature/sensorManager/MPU9250.h"

// Configuration l'adresse I2C (par défaut : 0x76 ou 0x77)
#define MY_BMP280_ADDRESS 0x76

EspNowHandler* espNowHandler;
GpioManager* gpioManager;
MotorManager* motorManager;
MPU9250* MPU9250Manager;

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpioManager = new GpioManager(GPIO_NUM_21, GPIO_NUM_2);
    if (!gpioManager) {
        ESP_LOGE("app_main", "Erreur allocation gpioManager");
        return;
    }
    gpioManager->init();

    espNowHandler = new EspNowHandler();
    if (!espNowHandler->init()) {
        ESP_LOGE("MAIN", "ESP-NOW init failed!");
        return;
    }

    motorManager = new MotorManager();
    motorManager->init();

    xTaskCreate([](void*) { motorManager->Task(); },
                "MotorManagerTask", 4096, &motorManager, 5, nullptr);
    
    MPU9250Manager = new MPU9250();
    xTaskCreate([](void*) { MPU9250Manager->Task(); },
                "MPU9250ManagerTask", 4096, &MPU9250Manager, 5, nullptr);
}

