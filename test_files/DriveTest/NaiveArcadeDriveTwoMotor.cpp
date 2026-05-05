#include "main.h"
#include "custom/config.hpp"

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true){
        int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        drive.arcade(throttle, steer);

       pros::delay(20);
    }
}