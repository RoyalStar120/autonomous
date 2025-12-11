#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "pros/apix.h"
#include "config.hpp"

class Intake {
public:
    Intake();

    // turn both intakes on (intake balls)
    void in(int power = 127);

    // reverse both (outtake)
    void out(int power = 127);

    // stop both
    void stop();

    // roller control (optional)
    void rollerIn(int power = 127);
    void rollerOut(int power = 127);
    void rollerStop();

private:
    pros::Motor bottomIntake;
    pros::Motor topIntake;
    pros::Motor roller;
};

#endif
