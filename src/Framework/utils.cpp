#include "custom/utils.hpp"

#include <cmath>

#include "custom/const.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

// Wrap Angle
// Wraps angles to [-180, 180]
double wrapAngle(double x)
{
    x = fmod(x + 180.0, 360.0);
    if (x < 0)
    {
        x += 360.0;
    }
    return x - 180.0;
}

// Angle Error
// Returns the shortest angle error for the robot/a mechanism to turn
double angleError(double target, double current)
{
    return wrapAngle(target - current);
}

// Joystick to Voltage Converting
int joystickToVoltage(int value)
{
    return value * 12000.0 / MAX_JOYSTICK;
}

// IMU Initialisation
void imuInit(pros::Imu& IMU)
{
    IMU.reset();

    while (IMU.is_calibrating())
    {
        pros::delay(20);
    }
}
