#ifndef GPIO_MANAGER_H
#define GPIO_MANAGER_H

#include "driver/gpio.h"

class GpioManager {
public:
    GpioManager(gpio_num_t ledPin, gpio_num_t wifiPin);
    void init();
    void setLed(bool state);
    void setWifi(bool state);

private:
    gpio_num_t ledPin;
    gpio_num_t wifiPin;
};

#endif // GPIO_MANAGER_H
