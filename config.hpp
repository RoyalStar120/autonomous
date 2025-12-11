#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "pros/apix.h"

// ----------------- PORTS -----------------

// Left side drive
inline constexpr int PORT_DRIVE_LF = -2;
inline constexpr int PORT_DRIVE_LM = -4;
inline constexpr int PORT_DRIVE_LB = -3;

// Right side drive
inline constexpr int PORT_DRIVE_RF = 9;
inline constexpr int PORT_DRIVE_RM = 8;
inline constexpr int PORT_DRIVE_RB = 7;

// Intakes
inline constexpr int PORT_INTAKE_BOTTOM = 6;  // CHANGE IF NEEDED
inline constexpr int PORT_INTAKE_TOP = 13;  // CHANGE IF NEEDED

// Roller / scorer
inline constexpr int PORT_ROLLER = 12;        // CHANGE IF NEEDED

// Optional IMU port
inline constexpr int PORT_IMU = 10;

// Controller
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// ----------------- MOTOR OBJECTS -----------------

// Drive motors
inline pros::Motor driveLF(PORT_DRIVE_LF);
inline pros::Motor driveLM(PORT_DRIVE_LM);
inline pros::Motor driveLB(PORT_DRIVE_LB);

inline pros::Motor driveRF(PORT_DRIVE_RF);
inline pros::Motor driveRM(PORT_DRIVE_RM);
inline pros::Motor driveRB(PORT_DRIVE_RB);

// Intake motors
inline pros::Motor intakeBottom(PORT_INTAKE_BOTTOM);
inline pros::Motor intakeTop(PORT_INTAKE_TOP);

// Roller (score mover)
inline pros::Motor roller(PORT_ROLLER);

// Drive power helper
inline void setDrivePower(double left, double right) {
    driveLF.move(left);
    driveLM.move(left);
    driveLB.move(left);

    driveRF.move(right);
    driveRM.move(right);
    driveRB.move(right);
}

// ----------------- GEOMETRY & CONSTANTS -----------------

inline constexpr double PI_VAL = 3.141592653589793;
inline constexpr double DRIVE_WHEEL_DIAMETER = 3.25; 
inline constexpr double DRIVE_WHEEL_CIRCUMFERENCE = PI_VAL * DRIVE_WHEEL_DIAMETER;
inline constexpr double INCHES_PER_DEGREE = DRIVE_WHEEL_CIRCUMFERENCE / 360.0;

inline constexpr double TRACK_WIDTH_INCHES = 11.0;  
inline constexpr double MAX_POWER = 127.0;

#endif // CONFIG_HPP
