#include "./feature/wifiManager/wifiManager.h"
#include "./feature/gpioManager/gpioManager.h"
#include "./feature/udpManager/udpManager.h"
#include "./feature/sensorManager/BMP280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

// Configuration Wi-Fi
#define WIFI_SSID      "ESP32_Hotspot"
#define WIFI_PASSWORD  "12345678"
#define UDP_PORT       1234
#define HOST_IP        "192.168.4.1"

// Configuration l'adresse I2C (par défaut : 0x76 ou 0x77)
#define MY_BMP280_ADDRESS 0x76

WifiManager* wifiManager;
GpioManager* gpioManager;
UdpManager* udpManager;
BMP280* bmp280;

extern "C" void app_main(void) {  // Utilisez "app_main" et non "appMain"
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialisation des modules
    gpioManager = new GpioManager(GPIO_NUM_21, GPIO_NUM_2);
    gpioManager->init();

    wifiManager = new WifiManager(WIFI_SSID, WIFI_PASSWORD);
    wifiManager->initSta();

    udpManager = new UdpManager(HOST_IP, UDP_PORT,gpioManager);
    udpManager->init();

    // Créer la tâche pour recevoir les messages UDP
    xTaskCreate([](void*) {
        udpManager->receiveTask(nullptr);
    }, "udp_receive_task", 4096, NULL, 5, NULL);
}
