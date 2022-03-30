#pragma once

#include <Arduino.h>

class pid
{
    public:
        pid(int PIN_1, int PIN_2, int ENC_PIN): _pin_1{PIN_1}, _pin_2{PIN_2}, _enc_pin{ENC_PIN}, _pwm_pin{PIN_1} {};
        void init();
        void tick(volatile long encoder_count);

        typedef enum {
            stop,
            back,
            forward,
            off
        } direction;

        void setTargetSpeed(int speed) { speed_setpoint = speed; };
        void setTargetDirection(direction direction);

        void setP(float p) { Kp = p; };
        void setI(float i) { Ki = i; };
        void setD(float d) { Kd = d; };

        void printMotorInfo();
    private:
        int _pin_1;
        int _pin_2;
        int _enc_pin;
        int _pwm_pin;

        long last_encoder_count = 0;

        int looptime = 20;                               // How often the loop gets executed

        unsigned long lastMilli = 0;                    // loop timing

        int speed_setpoint = 0;
        int speed_actual = 0;

        int PWM_val = 0;

        int last_error = 0;
        int accumulated_error = 0;

        // http://www.pcbheaven.com/wikipages/PID_Theory/?p=1
        // Kc: 6.9
        // Pc: 0.285
        // Kp = 0.60 x Kc => 4.14
        // Ki = 0.5 x Pc => 0.1425
        // Kd = Pc / 8 => 0.035625  

        float Kp = 0.9;//2;// Old: 0.9;                                // PID proportional control Gain
        float Ki = 0.0;//0.5;// Old: 0.0;                                // PID integral control Gain
        float Kd = 1.4;//1.5;// Old: 0.1;                                // PID derivative control Gain

        void getMotorData(volatile long encoder_count);
        void calculateNewPwmValue();
};
