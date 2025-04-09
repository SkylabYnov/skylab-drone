#include "MotorManager.h"

SemaphoreHandle_t MotorManager::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorManager::currentControllerRequestDTO;

MotorManager::MotorManager() {}

void MotorManager::init() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_timer_config_t timerConfig = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_12_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = PWM_FREQ,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false
        };
        ledc_timer_config(&timerConfig);

        ledc_channel_config_t channelConfig = {
            .gpio_num = escPins[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = ledcChannels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = PWM_MIN,
            .hpoint = 0,
            .flags = {0}
        };
        ledc_channel_config(&channelConfig);
    }
    ESP_LOGI(TAG, "Moteurs initialisés");
}

void MotorManager::setMotorSpeed(int motorIndex, int speed) {
    if(isEmergencyStop){
        return;
    }
    if (motorIndex < 0 || motorIndex >= NUM_MOTORS) {
        ESP_LOGW(TAG, "Index moteur invalide: %d", motorIndex);
        return;
    }
    speed = std::max(0, std::min(speed, 180));
    int duty = calcMotorDuty(speed);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledcChannels[motorIndex], duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledcChannels[motorIndex]);

    ESP_LOGI(TAG, "Moteur %d réglé à la vitesse %d (duty: %d)", motorIndex, speed, duty);
}

void MotorManager::emergencyStop()
{
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        setMotorSpeed(i,0);
    }
    isEmergencyStop = true;
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
                setMotorSpeed(i, (int)motorSpeeds[i]);
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
        motorSpeeds[i] = std::clamp(motorSpeeds[i] + throttleInput * 0.5f, 0.0f, 180.0f);
    }
}
 

int MotorManager::calcMotorDuty(int speed)
{
    return PWM_MIN + ((PWM_ESC_MAX - PWM_MIN) * speed) / 180;
}
