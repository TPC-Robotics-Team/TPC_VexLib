#pragma once
#include <cmath>

constexpr double DEG_TO_RAD = M_PI / 180;
constexpr double MAX_JOYSTICK = 127.0;

struct Pose
{
    double x;
    double y;
    double heading;
};