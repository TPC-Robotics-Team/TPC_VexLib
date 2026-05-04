#include "custom/state_estimation.hpp"

// Constructor
EncoderOdometry::EncoderOdometry(
    pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, double wheelDiam, double trackWidth, double gearRatio)
    : m_leftMotorGroup(leftMotors), m_rightMotorGroup(rightMotors), m_wheelCirc(wheelDiam * M_PI), m_trackWidth(trackWidth),
      m_gearRatio(gearRatio)
{
}


