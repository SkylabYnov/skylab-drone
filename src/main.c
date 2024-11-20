#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/gpio.h"

// Configuration Wi-Fi
#define WIFI_SSID      "ESP32_Hotspot"
#define WIFI_PASSWORD  "12345678"
#define UDP_PORT       1234
#define LED_PIN        21
#define WIFI_PIN       2
#define HOST_IP        "192.168.4.1"

static const char *TAG = "ESP32_UDP";
bool etatLed = false;
int sock;
struct sockaddr_in server_addr;

// Gestionnaire d'événements
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Connexion perdue, reconnexion...");
        gpio_set_level(WIFI_PIN, 0);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        gpio_set_level(WIFI_PIN, 1);
        ESP_LOGI(TAG, "Connecté avec IP: " IPSTR, IP2STR(&event->ip_info.ip));

        // Envoi initial du message "hello world !" au serveur
        struct sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_PORT);
        inet_pton(AF_INET, HOST_IP, &dest_addr.sin_addr.s_addr);
        sendto(sock, "hello world !", strlen("hello world !"), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    }
}

// Initialisation du Wi-Fi en mode station
void wifi_init_sta() {
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connexion au réseau Wi-Fi %s en cours...", WIFI_SSID);
}

// Fonction de création du socket UDP
void my_udp_init() {
    struct sockaddr_in addr;
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Erreur de création du socket : errno %d", errno);
        return;
    }

    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Erreur lors du bind : errno %d", errno);
        close(sock);
        return;
    }

    ESP_LOGI(TAG, "Serveur UDP prêt sur le port %d", UDP_PORT);
}

// Fonction de gestion des messages UDP
void udp_receive_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "Erreur de réception : errno %d", errno);
            break;
        } else {
            rx_buffer[len] = '\0'; // Terminer la chaîne
            ESP_LOGI(TAG, "Message reçu : %s", rx_buffer);

            if (strcmp(rx_buffer, "l1") == 0) {
                etatLed = !etatLed;
                if (etatLed) {
                    gpio_set_level(LED_PIN, 1);
                    sendto(sock, "led on", strlen("led on"), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                } else {
                    gpio_set_level(LED_PIN, 0);
                    sendto(sock, "led off", strlen("led off"), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                }
            }
        }
    }
    vTaskDelete(NULL);
}

// Fonction de gestion du Wi-Fi et UDP
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(WIFI_PIN, GPIO_MODE_OUTPUT);

    // Connexion au Wi-Fi
    wifi_init_sta();

    // Initialiser le socket UDP
    my_udp_init();

    // Créer la tâche pour recevoir les messages UDP
    xTaskCreate(udp_receive_task, "udp_receive_task", 4096, NULL, 5, NULL);
}
