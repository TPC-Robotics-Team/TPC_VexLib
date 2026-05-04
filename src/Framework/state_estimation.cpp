#include "custom/state_estimation.hpp"

// Constructor
EncoderOdometry::EncoderOdometry(pros::MotorGroup& leftMotors,
                                 pros::MotorGroup& rightMotors,
                                 double wheelDiam,
                                 double lastLeft,
                                 double lastRight,
                                 double lastTime,
                                 double track_width,
                                 double gearRatio)
    : m_leftMotorGroup(leftMotors), m_rightMotorGroup(rightMotors), m_wheelCirc(wheelDiam * M_PI), m_lastLeft(0.0),
      m_lastRight(0.0), m_last_time(pros::millis()), m_track_width(track_width), m_gearRatio(gearRatio)
{
}
