#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "driver/mcpwm.h"
#include "esp_log.h"
#include <algorithm>
#include <ControllerRequestDTO.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NUM_MOTORS 4
#define TAG "MotorManager"

class MotorManager {
public:
    MotorManager();
    void init();
    void setMotorSpeed(int motorIndex, float speed); // vitesse: 0 - 180
    void emergencyStop();
    void Task();

    static SemaphoreHandle_t xControllerRequestMutex;
    static ControllerRequestDTO currentControllerRequestDTO;

private:
    const int escPins[NUM_MOTORS] = {13, 12, 14, 15};  // GPIOs utilisés pour les moteurs
    const mcpwm_operator_t mcpwmOperators[NUM_MOTORS] = {MCPWM_OPR_A, MCPWM_OPR_B, MCPWM_OPR_A, MCPWM_OPR_B};

    static constexpr int PWM_FREQ = 50;     // 50Hz pour les ESC
    static constexpr int PWM_RES = 12;      // Résolution de 12 bits
    static constexpr int PWM_MAX = (1 << PWM_RES) - 1;
    static constexpr int PWM_MIN = int(PWM_MAX * 0.05);     // ~5% duty cycle (1000us)
    static constexpr int PWM_ESC_MAX = int(PWM_MAX * 0.10); // ~10% duty cycle (2000us)

    float motorSpeeds[NUM_MOTORS] = {0, 0, 0, 0};

    bool isEmergencyStop = false;
    
    void updateThrottle(float throttleInput);
    int calcMotorDuty(int speed);
};

#endif // MOTOR_MANAGER_H