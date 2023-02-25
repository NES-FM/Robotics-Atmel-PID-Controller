#pragma once

#define comm_v2

#include <Wire.h>
#include "../../include/i2c_registers.h"
#include "pid.h"

#ifndef comm_v2
class comm
{
    public:
        comm() {};
        void init(pid* m1, pid* m2);
        void tick();
    private:
        static void receiveEvent(int howMany);
        static bool received;

        static byte i2c_command;
        static byte i2c_value;

        pid* motor_1;
        pid* motor_2;
};
#else
class comm
{
    public:
        comm() {};
        void init(pid* m1, pid* m2);
        void tick();
        void receiveEvent(int howMany);
    private:
        pid* both_motors[2];

        struct stop_command {
            uint8_t motor_num;        // + 1 byte
            uint8_t type;             // + 1 byte
                                      // = 2 bytes
        };

        struct drive_command {
            uint8_t motor_num;        // + 1 byte
            uint8_t speed;            // + 1 byte
            uint8_t direction;        // + 1 byte
                                      // = 3 bytes
        };

        struct move_steps_command {
            uint8_t motor_num;        // + 1 byte
            uint8_t speed;            // + 1 byte
            uint8_t direction;        // + 1 byte
            uint16_t steps;           // + 3 byte
                                      // = 6 bytes
        };

        stop_command _rx_stop_command[2];
        drive_command _rx_drive_command[2];
        move_steps_command _rx_move_steps_command[2];

        enum received_type {
            received_none,
            received_stop,
            received_drive,
            received_move_steps
        };

        received_type _rx_received_type[2] = {received_none};
};

#endif
