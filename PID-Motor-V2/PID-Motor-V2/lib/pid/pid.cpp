#include "pid.h"

pid::pid(int PIN_1, int PIN_2, int ENC_PIN, bool inverted)
{
    _inverted = inverted;
    if (inverted)
    {
        _pin_1 = PIN_2;
        _pin_2 = PIN_1;
    }
    else
    {
        _pin_1 = PIN_1;
        _pin_2 = PIN_2;
    }
    _enc_pin = ENC_PIN;
    _pwm_pin = PIN_1;
}

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
    if (!(current_mode == currently_stopped || current_mode == currently_stopped_from_moving_steps_reached_limit) && ((millis() - lastMilli) >= looptime))  // Execute every looptime milliseconds
    {
        lastMilli = millis();
        getMotorData(encoder_count);  // Calculate Current Speed
        calculateNewPwmValue();
        analogWrite(_pwm_pin, PWM_val);

        // printMotorInfo();
        // Serial.println();
    }
}

void pid::getMotorData(volatile long encoder_count)
{
    speed_actual = ((encoder_count - last_encoder_count) * (60 * (1000 / looptime))) / 360;  // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
    last_encoder_count = encoder_count;
}

void pid::calculateNewPwmValue()
{
    if (first_move_after_stop)
    {
        first_move_after_stop = false;
        return;
    }
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

void pid::setStop(int stop_type)
{
    if (stop_type == stop_type_stop)
    {
        setTargetDirection(direction::stop);
    }
    else if (stop_type == stop_type_off)
    {
        setTargetDirection(direction::off);
    }
    current_mode = currently_stopped;
    first_move_after_stop = true;
}

void pid::setDrive(int drive_direction, int speed)
{
    setTargetSpeed(speed);

    if (drive_direction == move_direction_forward)
    {
        setTargetDirection(direction::forward);
    }
    else if (drive_direction == move_direction_backward)
    {
        setTargetDirection(direction::back);
    }
    current_mode = currently_moving_speed;
}


void pid::printMotorInfo()
{
    Serial.printf("SP: %d, RPM: %d, PWM: %d, PWM_PIN: %d", speed_setpoint, speed_actual, PWM_val, _pwm_pin);
    // Serial.printf("Speed_Setpoint:%d Current_Speed:%d PWM_Value:", speed_setpoint, speed_actual);
    // Serial.print(((float)PWM_val)/10);
    // Serial.print(" Kp:"); Serial.print(Kp);
    // Serial.print(" Ki:"); Serial.print(Ki);
    // Serial.print(" Kd:"); Serial.print(Kd);
}
