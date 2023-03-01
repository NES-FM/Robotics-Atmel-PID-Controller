#include "ard_comm.h"

#ifdef comm_v1
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
#endif

#if defined(comm_v2) || defined(comm_v3)
void comm::init(pid* m1, pid* m2)
{
    Wire.begin(I2C_ADDRESS);

    both_motors[0] = m1;
    both_motors[1] = m2;
}
#endif

#ifdef comm_v2
uint8_t motor_id = 0;

void comm::receiveEvent(int howMany)
{
    // if (Serial.available() > 0)
    // { 
        Serial.print("Received Wire event of size: "); Serial.println(howMany); 
    // }

    switch (howMany)
    {
        case sizeof(stop_command):
            Wire.readBytes(&motor_id, 1);
            Wire.readBytes((uint8_t*) &_rx_stop_command[motor_id-1]+1, sizeof(stop_command)-1);
            _rx_received_type[motor_id-1] = received_stop;
        break;
        case sizeof(drive_command):
            Wire.readBytes(&motor_id, 1);
            Wire.readBytes((uint8_t*) &_rx_drive_command[motor_id-1]+1, sizeof(drive_command)-1);
            _rx_received_type[motor_id-1] = received_drive;

        break;
        // case sizeof(move_steps_command):
        //     uint8_t motor_id = 0;
        //     Wire.readBytes(&motor_id, 1);

        // break;

        // case sizeof(stop_command):
        //     Wire.readBytes((byte*) &_rx_stop_command, sizeof(stop_command));

        //     // if (Serial.available() > 0)
        //     {
        //         Serial.print("Stopping with Motor_Num: "); Serial.print(_rx_stop_command.motor_num); Serial.print(" Type: "); Serial.println(_rx_stop_command.type); 
        //     }

        //     both_motors[_rx_stop_command.motor_num - 1]->setStop(_rx_stop_command.type);

        //     break;
        // case sizeof(drive_command):
        //     Wire.readBytes((byte*) &_rx_drive_command, sizeof(drive_command));

        //     // if (Serial.available() > 0)
        //     { 
        //         Serial.print("Driving with Motor_Num: "); Serial.print(_rx_drive_command.motor_num); Serial.print(" Speed: "); Serial.print(_rx_drive_command.speed); Serial.print(" Direction: "); Serial.println(_rx_drive_command.direction); 
        //     }

        //     both_motors[_rx_drive_command.motor_num - 1]->setDrive(_rx_drive_command.direction, _rx_drive_command.speed);

        //     break;
        // case sizeof(move_steps_command):
        //     Serial.println("aaaaaaah");
        //     Wire.readBytes((byte*) &_rx_move_steps_command, 6);  //sizeof(move_steps_command));

        //     // if (Serial.available() > 0)
        //     { 
        //         Serial.print("Move Steps with Motor_Num: "); Serial.print(_rx_move_steps_command.motor_num); Serial.print(" Speed: "); Serial.print(_rx_move_steps_command.speed); Serial.print(" Direction: "); Serial.print(_rx_move_steps_command.direction); Serial.print(" Steps: "); Serial.println(_rx_move_steps_command.steps); 
        //     }

        //     both_motors[_rx_drive_command.motor_num - 1]->setSteps(_rx_move_steps_command.direction, _rx_move_steps_command.speed, _rx_move_steps_command.steps);

        //     Serial.println("bbbbbbh");

        //     break;
        // default:
        //     break;
    }    
}

void comm::tick()
{
    for (int i = 0; i < 2; i++)
    {
        switch (_rx_received_type[i])
        {
            case received_drive:
            both_motors[i]->setDrive(_rx_drive_command[i].direction, _rx_drive_command[i].speed);
            break;
            case received_stop:
            both_motors[i]->setStop(_rx_stop_command[i].type);
            break;
            // case received_move_steps:
            // break;
            default:
            break;
        }
        _rx_received_type[i] = received_none;
    }
}
#endif

#ifdef comm_v3
void comm::receiveEvent(int howMany)
{
    buffer[head] = Wire.read();
    head = (head + 1) % BUFFER_SIZE;
    count++;

    if (Wire.available() > 0)
        this->receiveEvent(Wire.available());
}

void comm::tick()
{
    while (count > 0) {
        uint8_t received_byte = buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        count--;
        
        // extract variables from byte
        uint8_t motor_num = (received_byte >> 7) & 1; // extract first bit
        uint8_t type = (received_byte >> 6) & 1; // extract second bit
        uint8_t speed = received_byte & 0x3F; // extract remaining 6 bits
        
        if (speed == 0)
            both_motors[motor_num]->setStop(type);
        else
            both_motors[motor_num]->setDrive(type, speed);
    }
}
#endif
