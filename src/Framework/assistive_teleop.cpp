#include "assistive_teleop.hpp"

#include <cmath>

// Joystick Deadband
// Makes inputs near zero turn into zero to eliminate small movements
double deadband(double value, double threshold)
{
    if (std::abs(value) < threshold)
        return 0;
    // Linear interpolation to ensure continuity
    double sign = (value > 0) ? 1.0 : -1.0;
    double t = (std::abs(value) - threshold) / (MAX_JOYSTICK - threshold);
    return sign * std::lerp(0.0, MAX_JOYSTICK, t);
}

// Cubic Input Shaping
// Shapes inputs to add weight to high speeds and control in lower speeds
double shapeInput(double input, double a)
{
    double x = input / MAX_JOYSTICK;
    double shaped = a * x * x * x + (1 - a) * x;
    return shaped * 127;
}