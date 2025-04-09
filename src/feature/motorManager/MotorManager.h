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
    void setMotorSpeed(int motorIndex, float speed); // Vitesse entre 0.0 et 1.0
    void emergencyStop();
    void resetEmergencyStop();
    void Task();

    static SemaphoreHandle_t xControllerRequestMutex;
    static ControllerRequestDTO currentControllerRequestDTO;

private:
    struct MotorPwmConfig {
        mcpwm_unit_t unit;
        mcpwm_timer_t timer;
        mcpwm_operator_t op;
        mcpwm_io_signals_t signal;
    };

    const int escPins[NUM_MOTORS] = {13, 12, 14, 15};
    const MotorPwmConfig motorPwmConfigs[NUM_MOTORS] = {
        { MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM0A },
        { MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM0B },
        { MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM1A },
        { MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM1B }
    };

    static constexpr int PWM_FREQ = 50;     // 50Hz pour les ESC
    static constexpr int PWM_RES = 12;
    static constexpr int PWM_MAX = (1 << PWM_RES) - 1;
    static constexpr int PWM_MIN = int(PWM_MAX * 0.05);     // 1000us
    static constexpr int PWM_ESC_MAX = int(PWM_MAX * 0.10); // 2000us

    float motorSpeeds[NUM_MOTORS] = {0};
    bool isEmergencyStop = false;

    void updateThrottle(float throttleInput);
};

#endif // MOTOR_MANAGER_H
