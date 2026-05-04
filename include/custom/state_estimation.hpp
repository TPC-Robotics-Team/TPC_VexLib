#pragma once
#include "cmath"
#include "const.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

// Odometry
class EncoderOdometry
{
  private:
    pros::MotorGroup& m_leftMotorGroup;
    pros::MotorGroup& m_rightMotorGroup;
    double m_wheelCirc; // cm
    double m_gearRatio;
    double m_lastLeft = 0.0;
    double m_lastRight = 0.0;
    double m_last_time = 0.0;
    double m_track_width;

  public:
    EncoderOdometry(pros::MotorGroup& leftMotors,
                    pros::MotorGroup& rightMotors,
                    double wheelDiam,
                    double lastLeft,
                    double lastRight,
                    double lastTime,
                    double trackWidth,
                    double gearRatio = 1.0);

    void update();
    void reset();
    Pose getPose();
    double getVelocity();
};

class TwoWheelOdometry
{
  private:
    pros::Rotation& m_parallel;
    pros::Rotation& m_perpendicular;
    pros::Imu& m_imu;

    double m_wheel_circ;

    double m_parallel_offset;
    double m_perpendicular_offset;

    double m_x = 0.0;
    double m_y = 0.0;
    double m_theta = 0.0;

    double m_last_parallel = 0.0;
    double m_last_perpendicular = 0.0;
    double m_last_theta = 0.0;

    double m_last_time = 0.0;

    bool m_initialized = false;

  public:
    TwoWheelOdometry(pros::Rotation& parallel,
                     pros::Rotation& perpendicular,
                     pros::Imu& imu,
                     double wheel_Diam,
                     double parallel_offset,
                     double perpendicular_offset);

    void update();
    void reset();
    Pose getPose();
    double getVelocity();
};