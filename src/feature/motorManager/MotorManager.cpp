#include "MotorManager.h"

SemaphoreHandle_t MotorManager::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorManager::currentControllerRequestDTO;

MotorManager::MotorManager() {}

void MotorManager::init() {
    ESP_LOGI(TAG, "Initialisation MCPWM...");

    for (int i = 0; i < NUM_MOTORS; i++) {
        const auto& config = motorPwmConfigs[i];

        mcpwm_gpio_init(config.unit, config.signal, escPins[i]);

        mcpwm_config_t pwm_config = {};
        pwm_config.frequency = PWM_FREQ;
        pwm_config.cmpr_a = 0;
        pwm_config.cmpr_b = 0;
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

        mcpwm_init(config.unit, config.timer, &pwm_config);
    }

    // Envoi signal de démarrage à 0 pour armer les ESCs
    for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorSpeed(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Initialisation terminée");
}

void MotorManager::setMotorSpeed(int motorIndex, float speed) {
    if (isEmergencyStop || motorIndex < 0 || motorIndex >= NUM_MOTORS) {
        return;
    }

    speed = std::clamp(speed, 0.0f, 1.0f);
    float duty = speed * (PWM_ESC_MAX - PWM_MIN) + PWM_MIN;

    const auto& config = motorPwmConfigs[motorIndex];
    mcpwm_set_duty(config.unit, config.timer, config.op, duty);
    mcpwm_set_duty_type(config.unit, config.timer, config.op, MCPWM_DUTY_MODE_0);

    //ESP_LOGI(TAG, "Moteur %d réglé à la vitesse %.4f (duty: %.4f)", motorIndex, speed, duty);
}

void MotorManager::emergencyStop() {
    isEmergencyStop = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        const auto& config = motorPwmConfigs[i];
        mcpwm_set_duty(config.unit, config.timer, config.op, 0);
    }
    ESP_LOGW(TAG, "Arrêt d'urgence activé !");
}

void MotorManager::resetEmergencyStop() {
    isEmergencyStop = false;
    ESP_LOGI(TAG, "Arrêt d'urgence désactivé");
}

void MotorManager::Task() {
    ControllerRequestDTO lastControllerRequestDTO;

    while (true) {
        
        ControllerRequestDTO controllerRequestDTO;

        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            controllerRequestDTO = currentControllerRequestDTO;
            xSemaphoreGive(xControllerRequestMutex);
        }

        if (controllerRequestDTO.buttonEmergencyStop && *controllerRequestDTO.buttonEmergencyStop) {
            ESP_LOGI(TAG, "Alerte : Bouton d'arrêt d'urgence !");
            emergencyStop();
        }

        if (controllerRequestDTO.buttonMotorState && *controllerRequestDTO.buttonMotorState) {
            ESP_LOGI(TAG, "Bouton motorState activé");
        }

        if (controllerRequestDTO.flightController) {
            lastControllerRequestDTO = controllerRequestDTO;
            ESP_LOGI(TAG, "%s", lastControllerRequestDTO.toString().c_str());
        }

        if (!isEmergencyStop && lastControllerRequestDTO.flightController &&
            !lastControllerRequestDTO.flightController->isFullZero()) {
            updateThrottle(lastControllerRequestDTO.flightController->throttle);
            for (int i = 0; i < NUM_MOTORS; i++) {
                setMotorSpeed(i, motorSpeeds[i]);
            }
        }

        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            currentControllerRequestDTO.~ControllerRequestDTO();
            xSemaphoreGive(xControllerRequestMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void MotorManager::updateThrottle(float throttleInput) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motorSpeeds[i] = std::clamp(motorSpeeds[i] + throttleInput * 0.001f, 0.0f, 1.0f);
    }
}
