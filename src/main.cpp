#include "./feature/gpioManager/gpioManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "./feature/espNowHandler/EspNowHandler.h"
#include "./feature/motorManager/MotorManager.h"

// Configuration l'adresse I2C (par défaut : 0x76 ou 0x77)
#define MY_BMP280_ADDRESS 0x76

EspNowHandler* espNowHandler;
GpioManager* gpioManager;
MotorManager* motorManager;

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

    EspNowHandler* espNow = new EspNowHandler();
    if (!espNow->init()) {
        ESP_LOGE("MAIN", "ESP-NOW init failed!");
        return;
    }

    motorManager = new MotorManager();
    motorManager->init();

    xTaskCreate([](void*) { motorManager->Task(); },
                "MotorManagerTask", 4096, &motorManager, 5, nullptr);
}


