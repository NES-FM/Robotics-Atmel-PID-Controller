#include "pid.h"

void pid::init()
{
    pinMode(_pin_1, OUTPUT);
    pinMode(_pin_2, OUTPUT);
    pinMode(_enc_pin, INPUT_PULLUP);

    digitalWrite(_pin_1, LOW);
    digitalWrite(_pin_2, LOW);

    analogWrite(_pwm_pin, 0);
}

void pid::tick(volatile long encoder_count)
{
    if ((millis() - lastMilli) >= looptime)  // Execute every looptime milliseconds
    {
        lastMilli = millis();
        getMotorData(encoder_count);  // Calculate Current Speed
        calculateNewPwmValue();
        analogWrite(_pwm_pin, PWM_val);
    }
}

void pid::getMotorData(volatile long encoder_count)
{
    speed_actual = ((encoder_count - last_encoder_count) * (60 * (1000 / looptime))) / 360;  // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
    last_encoder_count = encoder_count;
}

void pid::calculateNewPwmValue()
{
    int error = abs(speed_setpoint) - abs(speed_actual);
    
    float PValue = error * Kp;

    float DValue = (error - last_error) * Kd;
    last_error = error;

    accumulated_error = (accumulated_error * 3 + error) / 4;
    float IValue = accumulated_error * Ki;

    PWM_val = constrain(PWM_val + int(PValue + IValue + DValue), 0, 255);
}

void pid::setTargetDirection(direction direction)
{
    switch (direction)
    {
        case stop:
            digitalWrite(_pin_1, HIGH);
            digitalWrite(_pin_2, HIGH);
            break;
            
        case back:
            digitalWrite(_pin_1, LOW);
            _pwm_pin = _pin_2;
            analogWrite(_pwm_pin, PWM_val);
            break;

        case forward:
            digitalWrite(_pin_2, LOW);
            _pwm_pin = _pin_1;
            analogWrite(_pwm_pin, PWM_val);
            break;

        case off:
            digitalWrite(_pin_1, LOW);
            digitalWrite(_pin_2, LOW);
            break;
    }
}

void pid::printMotorInfo()
{
    Serial.printf("SP: %d, RPM: %d, PWM: %d, PWM_PIN: %d", speed_setpoint, speed_actual, PWM_val, _pwm_pin);
}
