#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

extern "C"
{
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
}

#include "features/PidManager/PidManager.h"
#include <ControllerRequestDTO.h>
#include "esp_log.h"
#include "esp_err.h"
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu9250.h"

#define NUM_MOTORS 4
#define TAG_MOTOR_MANAGER "MotorManager"

class MotorManager
{
public:
    MotorManager();
    ~MotorManager();
    bool init(MPU9250 *imu);

    /*
    * @brief Set the speed of a motor.
    * @param motorIndex The index of the motor (0 to 3).
    * @param speed The speed value (0.0 to 1.0).
    * @return ESP_OK on success, or an error code on failure.
    */
    void setMotorSpeed(int motorIndex, float speed);
    void disarmMotors();
    void armMotors();
    void Task();

    static SemaphoreHandle_t xControllerRequestMutex;
    static ControllerRequestDTO currentControllerRequestDTO;

private:
    struct MotorPwmConfig
    {
        mcpwm_timer_handle_t timer;
        mcpwm_oper_handle_t operator_handle;
        mcpwm_cmpr_handle_t comparator;
        mcpwm_gen_handle_t generator;
    };

    const int escPins[NUM_MOTORS] = {
        26, // Avant gauche
        17, // Avant droit
        16, // Arrière droit
        27  // Arrière gauche
    };
    MotorPwmConfig motorPwmConfigs[NUM_MOTORS];

    static constexpr int PWM_FREQ_HZ = 50;                                      // 50Hz for ESCs
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;                    // 1MHz resolution
    static constexpr uint32_t PERIOD_TICKS = TIMER_RESOLUTION_HZ / PWM_FREQ_HZ; // 20000 ticks for 20ms period
    static constexpr uint32_t MIN_PULSE_TICKS = 1000;                           // 1000µs pulse width (idle)
    static constexpr uint32_t MAX_PULSE_TICKS = 2000;                           // 2000µs pulse width (full throttle)
    static constexpr uint32_t MAX_ANGLE = 30; // Maximum angle for roll and pitch in degrees
    static constexpr uint32_t MAX_YAW_RATE = 45; // Maximum yaw rate in degrees per second

    static constexpr float pkp = 1.0f;  // Proportional gain
    static constexpr float pki = 0.0f;  // Integral gain
    static constexpr float pkd = 0.05f; // Derivative gain
    static constexpr float rkp = 1.0f;  // Proportional gain
    static constexpr float rki = 0.0f;  // Integral gain
    static constexpr float rkd = 0.05f; // Derivative gain
    static constexpr float yawkp = 1.0f;  // Proportional gain
    static constexpr float yawki = 0.0f;  // Integral gain
    static constexpr float yawkd = 0.05f; // Derivative gain

    uint32_t motorSpeeds[NUM_MOTORS] = {0};
    bool isMotorArmed = false;

    PidManager pidPitch{pkp, pki, pkd}; // PID controller for pitch
    PidManager pidRoll{rkp, rki, rkd};  // PID controller for roll
    PidManager pidYaw{rkp, rki, rkd};  // PID controller for roll

    MPU9250 *imu;

    void updateThrottle(float throttleInput);
};

#endif