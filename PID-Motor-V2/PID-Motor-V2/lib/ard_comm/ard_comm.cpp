#include "ard_comm.h"

bool comm::received = false;
byte comm::i2c_command = 0x00;
byte comm::i2c_value = 0x00;

void comm::init(pid* m1, pid* m2)
{
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);

    motor_1 = m1;
    motor_2 = m2;
}

void comm::receiveEvent(int howMany)
{
    while (1 < Wire.available()) { // loop through all but the last
        i2c_command = Wire.read(); // receive byte as a character
    }
    i2c_value = Wire.read();    // receive byte as an integer
    
    if (Serial.available())
    {
        Serial.print("Received: "); Serial.print(i2c_command); Serial.print(" "); Serial.println(i2c_value);
    }

    received = true;
}

void comm::tick()
{
    if (received)
    {
        switch (i2c_command) {
            case MOTOR_1_SPEED:
                motor_1->setTargetSpeed(int(i2c_value));
                break;
            case MOTOR_1_DIREC:
                switch (i2c_value) {
                case MOTOR_DIREC_S:
                    motor_1->setTargetDirection(motor_1->stop);
                    break;
                case MOTOR_DIREC_B:
                    motor_1->setTargetDirection(motor_1->back);
                    break;
                case MOTOR_DIREC_F:
                    motor_1->setTargetDirection(motor_1->forward);
                    break;
                case MOTOR_DIREC_O:
                    motor_1->setTargetDirection(motor_1->off);
                    break;
                }
                break;

            case MOTOR_2_SPEED:
                motor_2->setTargetSpeed(int(i2c_value));
                break;
            case MOTOR_2_DIREC:
                switch (i2c_value) {
                case MOTOR_DIREC_S:
                    motor_2->setTargetDirection(motor_2->stop);
                    break;
                case MOTOR_DIREC_B:
                    motor_2->setTargetDirection(motor_2->back);
                    break;
                case MOTOR_DIREC_F:
                    motor_2->setTargetDirection(motor_2->forward);
                    break;
                case MOTOR_DIREC_O:
                    motor_2->setTargetDirection(motor_2->off);
                    break;
                }
                break;
        }
    }
}
