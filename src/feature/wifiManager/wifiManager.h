#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_wifi.h"
#include "esp_event.h"
#include <string>

class WifiManager {
public:
    WifiManager(const char* ssid, const char* password);
    ~WifiManager();
    
    void initSta();
    static void eventHandler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventData);

private:
    const char* ssid;
    const char* password;
};

#endif // WIFI_MANAGER_H
