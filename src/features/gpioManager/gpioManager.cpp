#include <features/GpioManager/GpioManager.h>

GpioManager::GpioManager(gpio_num_t ledPin, gpio_num_t wifiPin)
    : ledPin(ledPin), wifiPin(wifiPin) {}

void GpioManager::init()
{
    gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(wifiPin, GPIO_MODE_OUTPUT);
}

void GpioManager::setLed(bool state)
{
    gpio_set_level(ledPin, state ? 1 : 0);
}

void GpioManager::setWifi(bool state)
{
    gpio_set_level(wifiPin, state ? 1 : 0);
}
