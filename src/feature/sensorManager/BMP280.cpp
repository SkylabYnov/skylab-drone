#include "BMP280.h"


BMP280::BMP280(Adafruit_BME280 bmp): bmp(bmp) {}

bool BMP280::init() {
    Wire.begin();

    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);

        if (Wire.endTransmission() == 0) {
            return true;
            }
            
    return false;
  }
}

float BMP280::ReadTemperature() {
    float temperature = bmp.readTemperature();  
    return temperature;
}

float BMP280::ReadTemperature() {
    float pression = bmp.readPressure() / 100.0F;
    return pression;
  }
float BMP280::ReadHumidity() {
    float humidite = bmp.readHumidity();
    return humidite;
}
