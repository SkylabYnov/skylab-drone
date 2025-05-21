#ifndef PIDMANAGER_H
#define PIDMANAGER_H

class PidManager
{
public:
    PidManager(float kp, float ki, float kd);

    float calculate(float setpoint, float measured, float dt);

private:
    float kp, ki, kd;
    float previousError;
    float integral;
};

#endif // PIDMANAGER_H
