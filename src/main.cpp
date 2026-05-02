#include "main.h"
#include "controller.hpp"
#include "pros/misc.h"
#include "drive.hpp"
#include "utils.hpp"
#include "auton.hpp"


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors ({1});

pros::MotorGroup RightMotors ({-2});

pros::MotorGroup Intake({4,-11});

DifferentialDrive drive(LeftMotors, RightMotors);
AutonDifferentialDrive auton_drive(LeftMotors, RightMotors);

SlewLimiter slewlimit(5,7);


FeedForward driveff(6000.0, 0.0, 0.0);

void initialize() {}

void disabled() {}

void competition_initialize() {}

// Autonomous function area
void autonomous() {
    TrapezoidalProfile profile ({
        .maxVelocity = 60.0,
        .maxAcceleration = 120.0
    });

    profile.generate({0.0, 0.0}, {50.0, 0.0});

    uint32_t startTime = pros::millis();
    while (true){
        double elapsed = (pros::millis() - startTime) / 1000.0;

        if (profile.isFinished(elapsed)){
            auton_drive.tank(0,0);
            break;
        }

        ProfileState setpoint = profile.sample(elapsed);
        
        double ff = driveff.calculate(setpoint.velocity, 0.0);
        auton_drive.tank(ff, ff);

        pros::delay(10);
    }
}

// Teleop function area
void opcontrol() {
    while (true){
        int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        bool quickTurn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            autonomous();
        };


        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            Intake.move_voltage(12000);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            Intake.move_voltage(-12000);
        } else {
            Intake.move_voltage(0);
        }

        throttle = deadband(throttle);
        steer = deadband(steer);

        throttle = slewlimit.update(throttle);

        drive.curvature(throttle, steer, quickTurn);

       pros::delay(20);
    }
}