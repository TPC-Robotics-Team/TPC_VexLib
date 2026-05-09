#pragma once
#include <cmath>

constexpr double DEG_TO_RAD = M_PI / 180;
constexpr double MAX_JOYSTICK = 127.0;
constexpr double MAX_MILIVOLTS = 12000.0;

struct Pose
{
    double x;
    double y;
    double heading;
};

static constexpr int   ARM_SPEED       = 100;   // RPM
static constexpr int   CLAW_OPEN_POS   = 0;     // degrees
static constexpr int   CLAW_CLOSED_POS = 90;    
static constexpr int   CLAW_HOLD_VOLT  = 20; 