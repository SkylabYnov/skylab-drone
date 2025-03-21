#include "MotorController.h"

SemaphoreHandle_t MotorController::xControllerRequestMutex = xSemaphoreCreateMutex();
ControllerRequestDTO MotorController::currentControllerRequestDTO;

MotorController::MotorController() {}

void MotorController::init() {
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

void MotorController::setMotorSpeed(int motorIndex, int speed) {
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

void MotorController::emergencyStop()
{
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        setMotorSpeed(i,0);
    }
    isEmergencyStop = true;
}

void MotorController::Task()
{
    while (true) {
        ControllerRequestDTO controllerRequestDTO;

        // Protection avec un mutex pour éviter des corruptions de mémoire
        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            controllerRequestDTO = MotorController::currentControllerRequestDTO;
            xSemaphoreGive(xControllerRequestMutex);
        }

        if (controllerRequestDTO.buttonEmergencyStop && *controllerRequestDTO.buttonEmergencyStop) {  
            ESP_LOGI(TAG, "Alerte : Bouton d'arrêt d'urgence activé !");
            emergencyStop();
        }
        
        if (controllerRequestDTO.buttonMotorState && *controllerRequestDTO.buttonMotorState) {  
            ESP_LOGI(TAG, "Alerte : Bouton buttonMotorState activé !");
        }     

        if (controllerRequestDTO.flightController) {  
            updateThrottle(controllerRequestDTO.flightController->throttle);
        
            // Appliquer la vitesse mise à jour aux moteurs
            for (int i = 0; i < NUM_MOTORS; i++) {
                setMotorSpeed(i, motorSpeeds[i]);
            }
        }
        

        // Protection avec un mutex pour éviter des corruptions de mémoire
        if (xSemaphoreTake(xControllerRequestMutex, portMAX_DELAY)) {
            MotorController::currentControllerRequestDTO.~ControllerRequestDTO();
            xSemaphoreGive(xControllerRequestMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void MotorController::updateThrottle(float throttleInput) {
    if (throttleInput > deadZone || throttleInput < -deadZone) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motorSpeeds[i] = std::clamp(motorSpeeds[i] + throttleInput * 1.0f, 0.0f, 180.0f);
        }
    } 
}
 

int MotorController::calcMotorDuty(int speed)
{
    return PWM_MIN + ((PWM_ESC_MAX - PWM_MIN) * speed) / 180;
}
