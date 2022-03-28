#include <Arduino.h>
#include "Pin_definitions.h"


#include "pid.h"
volatile long motor_l_encoder_count = 0;
volatile long motor_r_encoder_count = 0;

void motor_left_isr() { motor_l_encoder_count++; }
void motor_right_isr() { motor_r_encoder_count++; }

pid motor_l(PIN_MOTOR_A_1, PIN_MOTOR_A_2, PIN_MOTOR_A_ENC);
pid motor_r(PIN_MOTOR_B_1, PIN_MOTOR_B_2, PIN_MOTOR_B_ENC);


#include "ard_comm.h"
comm comunication;


unsigned long lastPrintMillis = 0;


void setup() 
{
    // Blink LED Once at boot
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);

    motor_l.init();
    motor_r.init();

    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_A_ENC), motor_left_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_B_ENC), motor_right_isr, FALLING);

    comunication.init(&motor_l, &motor_r);
}

void loop() {
    if ((millis() - lastPrintMillis) >= 100 && Serial.available())
    {
        lastPrintMillis = millis();
        motor_l.printMotorInfo();
        Serial.print("\t");
        motor_r.printMotorInfo();
        Serial.println("");
    }

    comunication.tick();

    motor_l.tick(motor_l_encoder_count);
    motor_r.tick(motor_r_encoder_count);
}
