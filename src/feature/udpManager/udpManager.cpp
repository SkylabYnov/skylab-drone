#include "udpManager.h"


static const char *TAG = "UdpManager";

UdpManager::UdpManager(const char* hostIp, uint16_t port,GpioManager* gpioManager) 
    : hostIp(hostIp), port(port), gpioManager(gpioManager), sock(-1) {}

UdpManager::~UdpManager() {
    if (sock >= 0) {
        close(sock);
    }
}

void UdpManager::init() {
    struct sockaddr_in addr;
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Erreur de création du socket : errno %d", errno);
        return;
    }

    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Erreur lors du bind : errno %d", errno);
        close(sock);
        return;
    }

    ESP_LOGI(TAG, "Serveur UDP prêt sur le port %d", port);
}

void UdpManager::sendMessage(const char* message) {
    struct sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);
    inet_pton(AF_INET, hostIp, &destAddr.sin_addr.s_addr);
    sendto(sock, message, strlen(message), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
    ESP_LOGI(TAG, "message send %s", message);
}

void UdpManager::receiveTask(void *pvParameters) {
    char rxBuffer[128];
    struct sockaddr_in sourceAddr;
    socklen_t socklen = sizeof(sourceAddr);

    while (1) {
        int len = recvfrom(sock, rxBuffer, sizeof(rxBuffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "Erreur de réception : errno %d", errno);
            break;
        } else {
            rxBuffer[len] = '\0'; // Terminer la chaîne
            ESP_LOGI(TAG, "Message reçu : %s", rxBuffer);
            cJSON* json = cJSON_Parse(rxBuffer);
            ControllerRequestDTO controllerRequestDTO = ControllerRequestDTO::fromJson(json);
            delete json;

            if(lastController.getCounter()>=controllerRequestDTO.getCounter()){
                ESP_LOGI(TAG, "Message plus ancient que celui deja utiliser");
                return;
            }
            
            cJSON* jsonObj = controllerRequestDTO.toJson();
            char* jsonAffichage = cJSON_PrintUnformatted(jsonObj);

            if (jsonAffichage) {
                ESP_LOGI(TAG, "cast json to controllerRequestDTO : %s", jsonAffichage);
            }

            delete jsonAffichage;
            cJSON_Delete(jsonObj);  // Libérer l'objet `cJSON*`

        }
    }
    vTaskDelete(NULL);
}
