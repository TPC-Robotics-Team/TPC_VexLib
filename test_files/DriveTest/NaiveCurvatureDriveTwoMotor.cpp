#include "main.h"
#include "custom/config.hpp"

void opcontrol()
{
    while (true)
    {
        drive.curvature();

        pros::delay(20);
    }
}