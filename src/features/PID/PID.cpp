#include <features/PID/PID.h>

PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previousError(0), integral(0) {}

float PID::calculate(float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}