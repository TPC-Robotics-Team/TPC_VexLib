#pragma once
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "cmath"


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


class EncoderOdometry{
private:
    pros::MotorGroup& m_motorGroup;
    double m_wheelCirc; // cm
    double m_gearRatio;
    double m_lastPosition = 0.0;
    
    double degreesToCm(double degrees);

public:
    EncoderOdometry(pros::MotorGroup& motors, double wheelDiam, double gearRatio = 1.0);

    double getPosition();
    double getDelta();
    void reset(); 
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

