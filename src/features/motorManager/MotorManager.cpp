#include "features/MotorManager/MotorManager.hpp"
#include <inttypes.h>
#include "esp_timer.h"

// Static member initializations
SemaphoreHandle_t MotorManager::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorManager::currentControllerRequestDTO;

MotorManager::MotorManager()
{
    // Initialize motor speeds to zero
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motorSpeeds[i] = 0.0f;
    }
}

MotorManager::~MotorManager()
{
    imu = nullptr;
}

// Function to initialize the motor manager
bool MotorManager::init(MPU9250 *imu)
{
    ESP_LOGI(TAG_MOTOR_MANAGER, "Initializing MCPWM...");
    this->imu = imu;

    // Define shared timers for each group
    mcpwm_timer_handle_t shared_timers[2] = {nullptr, nullptr};

    for (int group_id = 0; group_id < 2; ++group_id)
    {
        // Create a shared timer for each group
        mcpwm_timer_config_t timer_config = {
            .group_id = group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = PERIOD_TICKS,
            .intr_priority = 0,
            .flags = {
                .update_period_on_empty = true,
                .update_period_on_sync = false,
                .allow_pd = false,
            }};
        if (mcpwm_new_timer(&timer_config, &shared_timers[group_id]) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create timer for group %d", group_id);
            return false;
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        int group_id = i / 2; // Use group 0 for motors 0,1 and group 1 for motors 2,3

        // Create operator configuration
        mcpwm_operator_config_t operator_config = {
            .group_id = group_id,
            .intr_priority = 0,
            .flags = {
                .update_gen_action_on_tez = false,
                .update_gen_action_on_tep = false,
                .update_gen_action_on_sync = false,
                .update_dead_time_on_tez = false,
                .update_dead_time_on_tep = false,
                .update_dead_time_on_sync = false,
            }};
        if (mcpwm_new_operator(&operator_config, &motorPwmConfigs[i].operator_handle) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create operator for motor %d", i);
            return false;
        }

        if (mcpwm_operator_connect_timer(motorPwmConfigs[i].operator_handle, shared_timers[group_id]) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to connect operator to timer for motor %d", i);
            return false;
        }

        // Create comparator configuration
        mcpwm_comparator_config_t comparator_config = {
            .intr_priority = 0,
            .flags = {
                .update_cmp_on_tez = true,
                .update_cmp_on_tep = false,
                .update_cmp_on_sync = false,
            }};
        if (mcpwm_new_comparator(motorPwmConfigs[i].operator_handle, &comparator_config, &motorPwmConfigs[i].comparator) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create comparator for motor %d", i);
            return false;
        }

        // Create generator configuration
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = escPins[i],
            .flags = {
                .invert_pwm = false,
                .io_loop_back = false,
                .io_od_mode = false,
                .pull_up = false,
                .pull_down = false,
            }};
        if (mcpwm_new_generator(motorPwmConfigs[i].operator_handle, &generator_config, &motorPwmConfigs[i].generator) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to create generator for motor %d", i);
            return false;
        }

        // Set generator actions
        if (mcpwm_generator_set_action_on_timer_event(
                motorPwmConfigs[i].generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    MCPWM_TIMER_EVENT_EMPTY,
                    MCPWM_GEN_ACTION_HIGH)) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to set generator action on timer event for motor %d", i);
            return false;
        }

        if (mcpwm_generator_set_action_on_compare_event(
                motorPwmConfigs[i].generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    motorPwmConfigs[i].comparator,
                    MCPWM_GEN_ACTION_LOW)) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to set generator action on compare event for motor %d", i);
            return false;
        }

        // Set initial duty cycle (idle)
        if (mcpwm_comparator_set_compare_value(motorPwmConfigs[i].comparator, (MIN_PULSE_TICKS + MAX_PULSE_TICKS) * 0.5) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to set initial compare value for motor %d", i);
            return false;
        }
    }

    // Synchronize the timers
    // Source timer for synchronization (Master timer, group 0)
    mcpwm_sync_handle_t sync_src = NULL;
    mcpwm_timer_sync_src_config_t sync_src_config = {
        .timer_event = MCPWM_TIMER_EVENT_EMPTY, // Trigger when timer count is zero
        .flags = {
            .propagate_input_sync = false,
        }};

    ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(shared_timers[0], &sync_src_config, &sync_src));

    // Phase timer synchronization (Slave timer, group 1)
    mcpwm_timer_sync_phase_config_t sync_config = {
        .sync_src = sync_src,
        .count_value = 0, // Start counting from 0 upon synchronization
        .direction = MCPWM_TIMER_DIRECTION_UP,
    };
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(shared_timers[1], &sync_config));

    // Enable and start timers
    for (int group_id = 0; group_id < 2; ++group_id)
    {
        if (mcpwm_timer_enable(shared_timers[group_id]) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to enable timer for group %d", group_id);
            return false;
        }
        if (mcpwm_timer_start_stop(shared_timers[group_id], MCPWM_TIMER_START_NO_STOP) != ESP_OK)
        {
            ESP_LOGE(TAG_MOTOR_MANAGER, "Failed to start timer for group %d", group_id);
            return false;
        }
    }

    // Send initial idle signal to arm ESCs
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        setMotorSpeed(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG_MOTOR_MANAGER, "Initialization complete");
    return true;
}

// Function to set motor speed
void MotorManager::setMotorSpeed(int motorIndex, float speed)
{
    if (!isMotorArmed || motorIndex < 0 || motorIndex >= NUM_MOTORS)
    {
        return;
    }

    speed = fmaxf(0.0f, fminf(1.0f, speed));

    uint32_t pulse_ticks = MIN_PULSE_TICKS + (uint32_t)((MAX_PULSE_TICKS - MIN_PULSE_TICKS) * speed);

    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(
            motorPwmConfigs[motorIndex].comparator,
            pulse_ticks));

    ESP_LOGI(
        TAG_MOTOR_MANAGER,
        "Motor %d set to speed %.4f (pulse width: %" PRIu32 " µs)",
        motorIndex,
        speed,
        pulse_ticks);
}

// Function to handle emergency stop
void MotorManager::disarmMotors()
{
    isMotorArmed = false;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motorSpeeds[i] = 0.0f;
        mcpwm_comparator_set_compare_value(motorPwmConfigs[i].comparator, MIN_PULSE_TICKS);
    }
    ESP_LOGW(TAG_MOTOR_MANAGER, "disable Motor Arming");
}

// Function to reset the emergency stop
void MotorManager::armMotors()
{
    isMotorArmed = true;
    ESP_LOGI(TAG_MOTOR_MANAGER, "enable Motor Arming; motors zeroed");
    // Optionally re‑arm ESCs by sending idle pulse for a moment:
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        setMotorSpeed(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Function call to start the task
void MotorManager::Task()
{
    ControllerRequestDTO lastControllerRequestDTO;
    MPU9250::Orientation currentOrientation;
    int64_t lastTime = esp_timer_get_time();

    while (true)
    {
        ControllerRequestDTO controllerRequestDTO;
        currentOrientation = imu->getOrientation(); // Get current orientation from MPU9250

        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY))
        {
            controllerRequestDTO = currentControllerRequestDTO;

            if (controllerRequestDTO.buttonMotorArming && !*controllerRequestDTO.buttonMotorArming)
            {
                if (*controllerRequestDTO.buttonMotorArming)
                {
                    armMotors();
                }
                else
                {
                    disarmMotors();
                }
            }

            if (controllerRequestDTO.buttonMotorState && *controllerRequestDTO.buttonMotorState)
            {
                ESP_LOGI(TAG_MOTOR_MANAGER, "Motor state button activated");
            }

            if (controllerRequestDTO.flightController)
            {
                lastControllerRequestDTO = controllerRequestDTO;
            }

            if (isMotorArmed && lastControllerRequestDTO.flightController &&
                !lastControllerRequestDTO.flightController->isFullZero())
            {
                // Compute dt (delta time) for PID calculations
                int64_t now = esp_timer_get_time();
                float dt = (now - lastTime) * 1e-6f;
                lastTime = now;

                // Correction PID
                float targetPitch = 0.0f; // drone doit rester stable
                float targetRoll = 0.0f;
                float targetYaw = 0.0f;

                float targetPitch = lastControllerRequestDTO.flightController->pitch;
                float targetRoll = lastControllerRequestDTO.flightController->roll;
                float targetYaw = lastControllerRequestDTO.flightController->yaw;

                float correctionPitch = pidPitch.calculate(targetPitch, currentOrientation.pitch, dt);
                float correctionRoll = pidRoll.calculate(targetRoll, currentOrientation.roll, dt);

                motorSpeeds[0] += correctionPitch + correctionRoll;  // Moteur 0 : avant-gauche
                motorSpeeds[1] += correctionPitch - correctionRoll;  // Moteur 1 : avant-droit
                motorSpeeds[2] += -correctionPitch - correctionRoll; // Moteur 2 : arrière-droit
                motorSpeeds[3] += -correctionPitch + correctionRoll; // Moteur 3 : arrière-gauche

                // Set motor speeds with clamping
                for (int i = 0; i < NUM_MOTORS; i++)
                {
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

// Function to update throttle based on input
void MotorManager::updateThrottle(float throttleInput)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motorSpeeds[i] = std::clamp(motorSpeeds[i] + throttleInput * 0.001f, 0.0f, 1.0f);
        ESP_LOGI(TAG_MOTOR_MANAGER, "Motor %d speed updated to %.4f", i, motorSpeeds[i]);
    }
}
