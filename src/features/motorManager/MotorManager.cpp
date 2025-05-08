#include <features/motorManager/MotorManager.h>
#include <inttypes.h>

// Static member initializations
SemaphoreHandle_t MotorManager::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorManager::currentControllerRequestDTO;

MotorManager::MotorManager() {
    // Initialize motor speeds to zero
    for (int i = 0; i < NUM_MOTORS; i++) {
        motorSpeeds[i] = 0.0f;
    }

    // Initialize emergency stop flag
    isEmergencyStop = false;
}

// Function to initialize the motor manager
bool MotorManager::init() {
    ESP_LOGI(TAG, "Initializing MCPWM...");

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Create timer configuration
        int group_id = i / 2;  // Use group 0 for motors 0,1 and group 1 for motors 2,3
        mcpwm_timer_config_t timer_config = {
            .group_id      = group_id,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = TIMER_RESOLUTION_HZ,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks  = PERIOD_TICKS,
            .intr_priority = 0,
            .flags = {
                .update_period_on_empty = true,
                .update_period_on_sync  = false,
            }
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motorPwmConfigs[i].timer));

        // Create operator configuration
        mcpwm_operator_config_t operator_config = {
            .group_id      = group_id,
            .intr_priority = 0,
            .flags = {
                .update_gen_action_on_tez  = false,
                .update_gen_action_on_tep  = false,
                .update_gen_action_on_sync = false,
                .update_dead_time_on_tez   = false,
                .update_dead_time_on_tep   = false,
                .update_dead_time_on_sync  = false,
            }
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motorPwmConfigs[i].operator_handle));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motorPwmConfigs[i].operator_handle, motorPwmConfigs[i].timer));

        // Create comparator configuration
        mcpwm_comparator_config_t comparator_config = {
            .intr_priority = 0,
            .flags = {
                .update_cmp_on_tez  = true,
                .update_cmp_on_tep  = false,
                .update_cmp_on_sync = false,
            }
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motorPwmConfigs[i].operator_handle, &comparator_config, &motorPwmConfigs[i].comparator));

        // Create generator configuration
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = escPins[i],
            .flags = {
                .invert_pwm    = false,
                .io_loop_back  = false,
                .io_od_mode    = false,
                .pull_up       = false,
                .pull_down     = false,
            }
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(motorPwmConfigs[i].operator_handle, &generator_config, &motorPwmConfigs[i].generator));

        // Set generator actions
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motorPwmConfigs[i].generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH
            )
        ));
        
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motorPwmConfigs[i].generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                motorPwmConfigs[i].comparator,
                MCPWM_GEN_ACTION_LOW
            )
        ));

        // Set initial duty cycle (idle)
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motorPwmConfigs[i].comparator, MIN_PULSE_TICKS));

        // Start MCPWM timer
        ESP_ERROR_CHECK(mcpwm_timer_enable(motorPwmConfigs[i].timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(motorPwmConfigs[i].timer, MCPWM_TIMER_START_NO_STOP));
    }

    // Send initial idle signal to arm ESCs
    for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorSpeed(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Initialization complete");
    return true;
}

// Function to set motor speed
void MotorManager::setMotorSpeed(int motorIndex, float speed) {
    if (isEmergencyStop || motorIndex < 0 || motorIndex >= NUM_MOTORS) {
        return;
    }

    uint32_t pulse_width_ticks = static_cast<uint32_t>(
        speed * (MAX_PULSE_TICKS - MIN_PULSE_TICKS) + MIN_PULSE_TICKS
    );

    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(
            motorPwmConfigs[motorIndex].comparator,
            pulse_width_ticks
        )
    );

    ESP_LOGI(
        TAG,
        "Motor %d set to speed %.4f (pulse width: %" PRIu32 " µs)",
        motorIndex,
        speed,
        pulse_width_ticks
    );
}

// Function to handle emergency stop
void MotorManager::emergencyStop() {
    isEmergencyStop = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motorSpeeds[i] = 0.0f;
        mcpwm_comparator_set_compare_value(..., MIN_PULSE_TICKS);
    }
    ESP_LOGW(TAG, "Emergency stop activated!");
}

// Function to reset the emergency stop
void MotorManager::resetEmergencyStop() {
    isEmergencyStop = false;
    ESP_LOGI(TAG, "Emergency stop deactivated; motors zeroed");
    // Optionally re‑arm ESCs by sending idle pulse for a moment:
    for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorSpeed(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Function call to start the task
void MotorManager::Task() {
    ControllerRequestDTO lastControllerRequestDTO;

    while (true) {
        ControllerRequestDTO controllerRequestDTO;

        Orientation currentOrientation;
        if (xSemaphoreTake(MPU9250::xOrientationMutex, pdMS_TO_TICKS(5))) {
            currentOrientation = MPU9250::orientation;
            xSemaphoreGive(MPU9250::xOrientationMutex);
        }


        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            controllerRequestDTO = currentControllerRequestDTO;

            if (controllerRequestDTO.buttonEmergencyStop && *controllerRequestDTO.buttonEmergencyStop) {
                ESP_LOGI(TAG, "Alert: Emergency stop button pressed!");
                emergencyStop();
            }

            if (controllerRequestDTO.buttonMotorState && *controllerRequestDTO.buttonMotorState) {
                ESP_LOGI(TAG, "Motor state button activated");
            }

            if (controllerRequestDTO.flightController) {
                lastControllerRequestDTO = controllerRequestDTO;
            }

            if (!isEmergencyStop && lastControllerRequestDTO.flightController &&
                !lastControllerRequestDTO.flightController->isFullZero()) {
            
                updateThrottle(lastControllerRequestDTO.flightController->throttle);
            
                // Correction PID
                float dt = 0.01f; // 10ms (vTaskDelay = 10ms)
                float targetPitch = 0.0f; // drone doit rester à plat
                float targetRoll = 0.0f;
            
                float correctionPitch = pidPitch.calculate(targetPitch, currentOrientation.pitch, dt);
                float correctionRoll = pidRoll.calculate(targetRoll, currentOrientation.roll, dt);
            
                // Moteur 0 : avant-gauche
                motorSpeeds[0] += correctionPitch + correctionRoll;

                // Moteur 1 : avant-droit
                motorSpeeds[1] += correctionPitch - correctionRoll;

                // Moteur 2 : arrière-droit
                motorSpeeds[2] += -correctionPitch - correctionRoll;
                
                // Moteur 3 : arrière-gauche
                motorSpeeds[3] += -correctionPitch + correctionRoll;

                // Set motor speeds with clamping
                for (int i = 0; i < NUM_MOTORS; i++) {
                    motorSpeeds[i] = std::clamp(motorSpeeds[i], 0.0f, 1.0f);
                    setMotorSpeed(i, motorSpeeds[i]);
                }
            }
            

            // Clear the shared DTO
            currentControllerRequestDTO.~ControllerRequestDTO();
            xSemaphoreGive(xControllerRequestMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void MotorManager::updateThrottle(float throttleInput) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motorSpeeds[i] = std::clamp(
            motorSpeeds[i] + throttleInput * 0.001f,
            0.0f,
            1.0f
        );
    }
}
