#pragma once

#include <Wire.h>
#include "../../include/i2c_registers.h"
#include "pid.h"

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
