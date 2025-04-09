#include "MotorManager.h"

SemaphoreHandle_t MotorManager::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorManager::currentControllerRequestDTO;

MotorManager::MotorManager() {}

void MotorManager::init() {
    ESP_LOGI(TAG, "Initialisation de MCPWM");

    // Tableau des signaux PWM correspondant aux moteurs
    const mcpwm_io_signals_t mcpwmSignals[NUM_MOTORS] = {
        MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B
    };

    for (int i = 0; i < NUM_MOTORS; i++) {
        // Configuration des GPIOs pour MCPWM
        mcpwm_gpio_init(MCPWM_UNIT_0, mcpwmSignals[i], escPins[i]);

        // Configuration du PWM
        mcpwm_config_t pwm_config = {};
        pwm_config.frequency = PWM_FREQ;
        pwm_config.cmpr_a = 0;
        pwm_config.cmpr_b = 0;
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

        // Initialisation du MCPWM pour chaque moteur
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    }

    ESP_LOGI(TAG, "MCPWM initialisé avec succès !");
}


void MotorManager::setMotorSpeed(int motorIndex, float speed) {
    if (isEmergencyStop) {
        return;
    }
    if (motorIndex < 0 || motorIndex >= NUM_MOTORS) {
        ESP_LOGW(TAG, "Index moteur invalide: %d", motorIndex);
        return;
    }
    speed = std::clamp(speed, 0.0f, 1.0f);
    float duty_cycle = float(speed) * (PWM_ESC_MAX - PWM_MIN) + PWM_MIN;

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, mcpwmOperators[motorIndex], duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, mcpwmOperators[motorIndex], MCPWM_DUTY_MODE_0);

    ESP_LOGI(TAG, "Moteur %d réglé à la vitesse %.4f (duty: %.4f)", motorIndex, speed, duty_cycle);
}

void MotorManager::emergencyStop() {
    isEmergencyStop = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, mcpwmOperators[i], 0);
    }
    ESP_LOGW(TAG, "Arrêt d'urgence activé !");
}

void MotorManager::Task()
{
    ControllerRequestDTO lastControllerRequestDTO; // Sauvegarde la dernière commande reçue

    while (true) {
        ControllerRequestDTO controllerRequestDTO;

        // Protection avec un mutex pour éviter des corruptions de mémoire
        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            controllerRequestDTO = MotorManager::currentControllerRequestDTO;
            xSemaphoreGive(xControllerRequestMutex);
        }

        // Vérification du bouton d'arrêt d'urgence
        if (controllerRequestDTO.buttonEmergencyStop && *controllerRequestDTO.buttonEmergencyStop) {  
            ESP_LOGI(TAG, "Alerte : Bouton d'arrêt d'urgence activé !");
            emergencyStop();
        }
        
        if (controllerRequestDTO.buttonMotorState && *controllerRequestDTO.buttonMotorState) {  
            ESP_LOGI(TAG, "Alerte : Bouton buttonMotorState activé !");
        }     

        // Si on reçoit une nouvelle commande, on met à jour la sauvegarde
        if (controllerRequestDTO.flightController) {  
            lastControllerRequestDTO = controllerRequestDTO;
        }

        // Appliquer la dernière valeur de throttle en continu
        if (lastControllerRequestDTO.flightController && !lastControllerRequestDTO.flightController->isFullZero()) {
            updateThrottle(lastControllerRequestDTO.flightController->throttle);
            for (int i = 0; i < NUM_MOTORS; i++) {
                setMotorSpeed(i, motorSpeeds[i]);
            }
            
        }

        // Protection avec un mutex pour éviter des corruptions de mémoire
        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            MotorManager::currentControllerRequestDTO.~ControllerRequestDTO();
            xSemaphoreGive(xControllerRequestMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void MotorManager::updateThrottle(float throttleInput) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motorSpeeds[i] = std::clamp(motorSpeeds[i] + throttleInput * 0.01f, 0.0f, 1.0f);
    }
}
 

int MotorManager::calcMotorDuty(int speed)
{
    return PWM_MIN + ((PWM_ESC_MAX - PWM_MIN) * speed) / 180;
}
