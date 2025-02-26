#ifndef UDP_MANAGER_H
#define UDP_MANAGER_H

#include <string>
#include <ControllerRequestDTO.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "../gpioManager/gpioManager.h"

class UdpManager {
public:
    UdpManager(const char* hostIp, uint16_t port,GpioManager* gpioManager);
    ~UdpManager();

    void init();
    void sendMessage(const char* message);
    void receiveTask(void *pvParameters);

private:
    const char* hostIp;
    uint16_t port;
    GpioManager* gpioManager;
    int sock;
    struct sockaddr_in serverAddr;

    ControllerRequestDTO lastController;
};

#endif // UDP_MANAGER_H
