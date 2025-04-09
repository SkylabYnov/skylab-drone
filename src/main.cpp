#include "driver/mcpwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_PWM_GPIO 18  // Le GPIO utilisé pour le signal PWM

extern "C" void app_main() {
    // 1. Initialisation du MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_GPIO);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;             // Fréquence PWM en Hz
    pwm_config.cmpr_a = 10;                   // Duty cycle du signal A en %
    pwm_config.cmpr_b = 0;                  // Duty cycle du signal B en %
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // PWM actif-haut
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // 2. Boucle pour faire tourner le moteur
    while (1) {
        // Monter progressivement la vitesse
        for (int duty = 0; duty <= 20; duty += 1) {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            ESP_LOGE("aled","%d",duty);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Descendre progressivement la vitesse
        for (int duty = 20; duty >= 0; duty -= 1) {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            ESP_LOGE("aled","%d",duty);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}



// #include "./feature/gpioManager/gpioManager.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_netif.h"
// #include "esp_event.h"
// #include "./feature/espNowHandler/EspNowHandler.h"
// #include "./feature/motorManager/MotorManager.h"
// #include "./feature/sensorManager/MPU9250.h"

// // Configuration l'adresse I2C (par défaut : 0x76 ou 0x77)
// #define MY_BMP280_ADDRESS 0x76

// EspNowHandler* espNowHandler;
// GpioManager* gpioManager;
// MotorManager* motorManager;
// MPU9250* MPU9250Manager;

// extern "C" void app_main(void) {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

//     gpioManager = new GpioManager(GPIO_NUM_21, GPIO_NUM_2);
//     if (!gpioManager) {
//         ESP_LOGE("app_main", "Erreur allocation gpioManager");
//         return;
//     }
//     gpioManager->init();

//     EspNowHandler* espNow = new EspNowHandler();
//     if (!espNow->init()) {
//         ESP_LOGE("MAIN", "ESP-NOW init failed!");
//         return;
//     }

//     motorManager = new MotorManager();
//     motorManager->init();

//     xTaskCreate([](void*) { motorManager->Task(); },
//                 "MotorManagerTask", 4096, &motorManager, 5, nullptr);
    
//     MPU9250Manager = new MPU9250();
//     // xTaskCreate([](void*) { MPU9250Manager->Task(); },
//     //             "MPU9250ManagerTask", 4096, &MPU9250Manager, 5, nullptr);
// }


