#include "main.h"

void initialize() {
    pros::lcd::initialize();;
}

void opcontrol() {
    while (true) {
        pros::lcd::print(0, "HELLO\n");
        pros::delay(500);
    }
}