#pragma once
#include "pros/imu.hpp"

constexpr double MAX_JOYSTICK = 127.0;
/* 
VEX V5 controller only receives values from -127 to 127. Use this constexpr when normalising input.
*/


double deadband(double value, double threshold = 5);
/*
Creates a zone of dead values [-threshold, threshold]. If input > threshold or input < threshold, input = 0. 

Uses linear interpolation to maintain continuity.
*/


double shapeInput(double input, double a = 0.5);
/*
Uses a cubic polynomial to shape the input such that it's smoother for the driver to control.
*/


double wrapAngle(double x);
/*
Wraps the angle of rotation to always be in [-360, 360]
*/


double angleError(double target, double current);
/*

Returns the absolute value of the angle error.

Already takes into account angle wrapping.

e.g:

angleError(350, 10) = 20
*/


void normalizeDriveTwoMotors(double &left, double &right);
/* 

Normalize Drive For Two-motor Arcade Drive

Maps [-254, 254] from kinematics to [-127, 127] for motors using scaling (maintains curvature)

*/


int joystickToVoltage(int value);
/*
Maps [-127, 127] to [-12000, 12000]
*/


void imuInit(pros::Imu &IMU);
/*
Initialises IMU sensor at the start
*/






