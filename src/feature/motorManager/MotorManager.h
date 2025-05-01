#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include <algorithm>
#include <ControllerRequestDTO.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../PID/PID.h"
#include "../sensorManager/MPU9250.h"

#define NUM_MOTORS 4
#define TAG "MotorManager"

class MotorManager {
public:
    MotorManager();
    void init();
    void setMotorSpeed(int motorIndex, float speed); // Speed between 0.0 and 1.0
    void emergencyStop();
    void resetEmergencyStop();
    void Task();

    static SemaphoreHandle_t xControllerRequestMutex;
    static ControllerRequestDTO currentControllerRequestDTO;

private:
    struct MotorPwmConfig {
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t operator_handle;
        mcpwm_cmpr_handle_t comparator;
        mcpwm_gen_handle_t generator;
    };

    PID pidPitch{1.0f, 0.0f, 0.05f}; 
    PID pidRoll{1.0f, 0.0f, 0.05f};


    const int escPins[NUM_MOTORS] = {13, 12, 14, 15};
    MotorPwmConfig motorPwmConfigs[NUM_MOTORS];

    static constexpr int PWM_FREQ_HZ = 50;       // 50Hz for ESCs
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000; // 1MHz resolution
    static constexpr uint32_t PERIOD_TICKS = TIMER_RESOLUTION_HZ / PWM_FREQ_HZ; // 20000 ticks for 20ms period
    static constexpr uint32_t MIN_PULSE_TICKS = 1000;  // 1000µs pulse width (idle)
    static constexpr uint32_t MAX_PULSE_TICKS = 2000;  // 2000µs pulse width (full throttle)

    float motorSpeeds[NUM_MOTORS] = {0};
    bool isEmergencyStop = false;

    void updateThrottle(float throttleInput);
};

#endif