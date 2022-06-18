#include "ard_comm.h"

#ifndef comm_v2
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
    // while (1 < Wire.available()) { // loop through all but the last
    //     i2c_command = Wire.read(); // receive byte as a character
    // }
    // i2c_value = Wire.read();    // receive byte as an integer
    
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
        while(Wire.available() > 1)
        {
            i2c_command = Wire.read();
            i2c_value = Wire.read();
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
}
#else
void comm::init(pid* m1, pid* m2)
{
    Wire.begin(I2C_ADDRESS);

    motor_1 = m1;
    motor_2 = m2;
}

void comm::receiveEvent(int howMany)
{
    if (Serial.available() > 0)
    { Serial.print("Received Wire event of size: "); Serial.println(howMany); }

    switch (howMany)
    {
        case sizeof(stop_command):
            _rx_received_type = received_stop;
            break;
        case sizeof(drive_command):
            _rx_received_type = received_drive;
            break;
        case sizeof(move_steps_command):
            _rx_received_type = received_move_steps;
            break;
        default:
            _rx_received_type = received_none;
            break;
    }    
}

void comm::tick()
{
    if (_rx_received_type != received_none)
    {
        switch (_rx_received_type)
        {
            case received_stop:
                Wire.readBytes((byte*) &_rx_stop_command, sizeof(stop_command));
                if (Serial.available() > 0)
                { Serial.print("Stopping with Motor_Num: "); Serial.print(_rx_stop_command.motor_num); Serial.print(" Type: "); Serial.println(_rx_stop_command.type); }
                break;
            case received_drive:
                Wire.readBytes((byte*) &_rx_drive_command, sizeof(drive_command));
                if (Serial.available() > 0)
                { Serial.print("Driving with Motor_Num: "); Serial.print(_rx_drive_command.motor_num); Serial.print(" Speed: "); Serial.print(_rx_drive_command.speed); Serial.print(" Direction: "); Serial.println(_rx_drive_command.direction); }
                break;
            case received_move_steps:
                Wire.readBytes((byte*) &_rx_move_steps_command, sizeof(move_steps_command));
                if (Serial.available() > 0)
                { Serial.print("Driving with Motor_Num: "); Serial.print(_rx_move_steps_command.motor_num); Serial.print(" Speed: "); Serial.print(_rx_move_steps_command.speed); Serial.print(" Direction: "); Serial.print(_rx_move_steps_command.direction); Serial.print(" Steps: "); Serial.println(_rx_move_steps_command.steps); }
                break;
            default:
                break;
        }
    }
}

#endif
