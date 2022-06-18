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
        pid* motor_1;
        pid* motor_2;
        
        #define stop_type_stop 0
        #define stop_type_off 1

        #define move_direction_forward 0
        #define move_direction_backward 1

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
            uint16_t steps;           // + 1 byte
                                      // = 4 bytes
        };

        stop_command _rx_stop_command;
        drive_command _rx_drive_command;
        move_steps_command _rx_move_steps_command;

        enum received_type {
            received_none,
            received_stop,
            received_drive,
            received_move_steps
        };

        received_type _rx_received_type = received_none;
};

#endif
