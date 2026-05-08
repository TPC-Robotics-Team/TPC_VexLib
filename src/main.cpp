#include "main.h"
#include "custom/config.hpp"


void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
    while (true)
    {
        drive.tank();

        pros::delay(20);
    }
}