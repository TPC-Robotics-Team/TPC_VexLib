#pragma once
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "cmath"
#include "pros/rotation.hpp"

constexpr double DEG_TO_RAD = M_PI / 180;


// Slew Delimitation
// Limits Acceleration to avoid high jerk
class SlewLimiter{
private:
    double accel;
    double decel;
    double prev;

public:
    SlewLimiter(double accel = 5, double decel = 10);

    double update(double input);
    void reset(double value = 0);
};



// PID controller
class PID{
private:
    double kP, kI, kD;
    double integral;
    double integralZone;
    double integralLimit;
    double prevMeasurement;
    double outputLimit;

public:
    PID(double p, double i, double d, double iZone, double iMax, double outputLimit);

    double update(double target, double current);
    bool isSettled();
    void reset();
};



// Heading Hold
// Maintains heading in autonomous and assistive teleOp
class HeadingHold{
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


// FeedForward controller
class FeedForward{
private:
    double kS, kV, kA;

    static double sign(double velocity);

public:
    FeedForward(double kS, double kV, double kA);

    double calculate(double velocity, double accel);
    
};


// Robot pose
struct Pose{
    double x;
    double y;
    double heading;
};



struct ProfileConstraints{
    double maxVelocity;
    double maxAcceleration;
};

struct ProfileState{
    double position;
    double velocity;
};



class TrapezoidalProfile{
private:
    ProfileConstraints m_constraints;
    ProfileState m_initial{};
    ProfileState m_goal{};

    double m_direction = 1.0;
    double m_t_accel = 0.0;
    double m_t_cruise = 0.0;
    double m_t_decel = 0.0;
    double m_totalTime = 0.0;
    bool m_generated = false;

public:
    TrapezoidalProfile(ProfileConstraints constraints);

    void generate(ProfileState initial, ProfileState goal);
    ProfileState sample(double t);
    bool isFinished(double t);
    double totalTime();
};


// Odometry
class EncoderOdometry{
private:
    pros::MotorGroup& m_leftMotorGroup;
    pros::MotorGroup& m_rightMotorGroup;
    double m_wheelCirc; // cm
    double m_gearRatio;
    double m_lastLeft = 0.0;
    double m_lastRight = 0.0;
    double m_last_time = 0.0;
    
    double degreesToCm(double degrees);

public:
    EncoderOdometry(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, double wheelDiam, double gearRatio = 1.0);

    double getPosition();
    double getDelta();
    double getVelocity();
    Pose getPose();
    void reset(); 
};

class TwoWheelOdometry{
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
    TwoWheelOdometry(
        pros::Rotation& parallel,
        pros::Rotation& perpendicular,
        pros::Imu& imu,
        double wheel_Diam,
        double parallel_offset,
        double perpendicular_offset
    );

    void update();
    void reset();
    double getX();
    double getY();
    double getTheta();
    Pose getPose();
    double getVelocity();
};
