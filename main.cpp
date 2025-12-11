#include "main.h"
#include "config.hpp"
#include "auton.hpp"
#include "drive.hpp"

// Instantiate Drive if you want to use it (not strictly necessary)
Drive drive;

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Init...");

    // Brake modes
    driveLF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveLM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveLB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    driveRF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    driveRB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    pros::lcd::set_text(1, "Init done");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    runMatchAuton();
}

void opcontrol() {
    // Arcade: left stick Y = forward/back, right stick X = turn
    while (true) {
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        int leftPower = forward + turn;
        int rightPower = forward - turn;

        // clamp
        if (leftPower > 127) leftPower = 127;
        if (leftPower < -127) leftPower = -127;
        if (rightPower > 127) rightPower = 127;
        if (rightPower < -127) rightPower = -127;

        setDrivePower(leftPower, rightPower);

        // Optionally update odom in background for telemetry:
        // odom.update();

        pros::delay(20);
    }
}
