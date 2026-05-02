#pragma once
#include "control.hpp"
#include "pros/imu.hpp"

/*
VEX V5 controller only receives values from -127 to 127. Use this constexpr when
normalising input.
*/

double deadband(double value, double threshold = 5);
/*
Creates a zone of dead values [-threshold, threshold]. If input > threshold or
input < threshold, input = 0.

Uses linear interpolation to maintain continuity.
*/

double shapeInput(double input, double a = 0.5);
/*
Uses a cubic polynomial to shape the input such that it's smoother for the
driver to control.
*/

// Heading Hold
// Maintains heading in autonomous and assistive teleOp
class HeadingHold {
private:
  pros::Imu &imu;
  PID controller;

  double targetHeading;
  bool enabled;

public:
  HeadingHold(pros::Imu &imu, PID pid);

  double update(double throttleInput, double steerInput);
  void setTarget(double target);
  void reset();
};