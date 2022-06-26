#include <Arduino.h>
#include "Pin_definitions.h"


#include "pid.h"
volatile long motor_l_encoder_count = 0;
volatile long motor_r_encoder_count = 0;

void motor_left_isr() { motor_l_encoder_count++; }
void motor_right_isr() { motor_r_encoder_count++; }

pid motor_l(PIN_MOTOR_A_1, PIN_MOTOR_A_2, PIN_MOTOR_A_ENC, true);
pid motor_r(PIN_MOTOR_B_1, PIN_MOTOR_B_2, PIN_MOTOR_B_ENC, true);


#include "ard_comm.h"
comm comunication; 
void wire_receive_event(int num_bytes) { comunication.receiveEvent(num_bytes); }

unsigned long lastPrintMillis = 0;

unsigned long lastChangeMillis = 0;
unsigned long lastChangeMillisFutureTime = 10000;
bool lastSpeed = false;

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
    motor_l.setStop(stop_type_stop);
    motor_r.setStop(stop_type_stop);
    
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_A_ENC), motor_left_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_MOTOR_B_ENC), motor_right_isr, FALLING);

    comunication.init(&motor_l, &motor_r);

    Wire.onReceive(wire_receive_event);

    lastChangeMillis = millis();
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

    // if ((millis() - lastChangeMillis) >= lastChangeMillisFutureTime)
    // {
    //     lastChangeMillis = millis();
    //     if (lastSpeed)
    //     {
    //         motor_l.setTargetSpeed(0);
    //         lastChangeMillisFutureTime = 3000;
    //     }
    //     else
    //     {
    //         motor_l.setTargetSpeed(40);
    //         lastChangeMillisFutureTime =10000;
    //     }
        
    //     lastSpeed = !lastSpeed;
    // }

    comunication.tick();

    motor_l.tick(motor_l_encoder_count);
    motor_r.tick(motor_r_encoder_count);

    // if (Serial.available())
    // {
    //     String message = Serial.readStringUntil('\n');
    //     if (message.startsWith("P"))
    //     {
    //         motor_l.setP(message.substring(1).toFloat());
    //     }
    //     else if (message.startsWith("I"))
    //         motor_l.setI(message.substring(1).toFloat());
    //     else if (message.startsWith("D"))
    //         motor_l.setD(message.substring(1).toFloat());
    // }
}
