#include <features/EspNowHandler/EspNowHandler.h>
#include <features/MotorManager/MotorManager.h>
#include <features/SensorManager/MPU9250.h>
#include <features/GpioManager/GpioManager.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

EspNowHandler *espNowHandler;
GpioManager *gpioManager;
MotorManager *motorManager;
MPU9250 *MPU9250Manager;

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize GPIO for LED and WiFi
    gpioManager = new GpioManager(GPIO_NUM_21, GPIO_NUM_2);
    if (!gpioManager)
    {
        ESP_LOGE("app_main", "Erreur allocation gpioManager");
        return;
    }
    gpioManager->init();

    // Initialize ESP-NOW
    espNowHandler = new EspNowHandler();
    if (!espNowHandler->init())
    {
        ESP_LOGE("MAIN", "ESP-NOW init failed!");
        return;
    }

    // Initialize ESP-NOW handlers
    motorManager = new MotorManager();
    if (!motorManager->init())
    {
        ESP_LOGE("MAIN", "MotorManager init failed!");
        return;
    }

    // Initialize MPU9250
    MPU9250Manager = new MPU9250();

    // Initialize tasks
    xTaskCreate([](void *)
                { motorManager->Task(); },
                "MotorManagerTask", 4096, &motorManager, 5, nullptr);

    xTaskCreate([](void *)
                { MPU9250Manager->Task(); },
                "MPU9250ManagerTask", 4096, &MPU9250Manager, 5, nullptr);
}
