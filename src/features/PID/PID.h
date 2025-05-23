class PID
{
public:
    PID(float kp, float ki, float kd);

    float calculate(float setpoint, float measured, float dt);

private:
    float kp, ki, kd;
    float previousError;
    float integral;
};
