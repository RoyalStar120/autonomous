#include "odom.hpp"
#include <cmath>

// Constructor
Odom::Odom()
    : xPos(0.0), yPos(0.0), headingRad(0.0),
      lastLeftDeg(0.0), lastRightDeg(0.0), lastUpdateMs(0) {}

// Average left encoder degrees
double Odom::getLeftDeg() const {
    return (driveLF.get_position() + driveLM.get_position() + driveLB.get_position()) / 3.0;
}

// Average right encoder degrees
double Odom::getRightDeg() const {
    return (driveRF.get_position() + driveRM.get_position() + driveRB.get_position()) / 3.0;
}

void Odom::reset(double x, double y, double heading_deg) {
    xPos = x;
    yPos = y;
    headingRad = heading_deg * PI_VAL / 180.0;

    // zero encoders
    driveLF.tare_position();
    driveLM.tare_position();
    driveLB.tare_position();
    driveRF.tare_position();
    driveRM.tare_position();
    driveRB.tare_position();

    lastLeftDeg = 0.0;
    lastRightDeg = 0.0;
    lastUpdateMs = pros::millis();
}

void Odom::update() {
    std::uint32_t now = pros::millis();
    double dt = (now - lastUpdateMs) / 1000.0;
    if (dt <= 0.0) return;

    double leftDeg  = getLeftDeg();
    double rightDeg = getRightDeg();

    double dLeftDeg  = leftDeg  - lastLeftDeg;
    double dRightDeg = rightDeg - lastRightDeg;

    lastLeftDeg = leftDeg;
    lastRightDeg = rightDeg;
    lastUpdateMs = now;

    double dLeftIn  = dLeftDeg  * INCHES_PER_DEGREE;
    double dRightIn = dRightDeg * INCHES_PER_DEGREE;

    double dCenter = (dLeftIn + dRightIn) / 2.0;
    double dTheta  = (dRightIn - dLeftIn) / TRACK_WIDTH_INCHES; // radians

    if (std::fabs(dTheta) < 1e-6) {
        // nearly straight
        xPos += dCenter * std::cos(headingRad);
        yPos += dCenter * std::sin(headingRad);
    } else {
        double r = dCenter / dTheta;
        double newHeading = headingRad + dTheta;
        xPos += r * (std::sin(newHeading) - std::sin(headingRad));
        yPos -= r * (std::cos(newHeading) - std::cos(headingRad));
        headingRad = newHeading;
    }
}

void Odom::driveTo(double targetX, double targetY, double targetHeadingDeg, double maxTimeSeconds) {
    PID drivePID(0.55, 0.0, 0.05, MAX_POWER, 1000.0);
    PID turnPID (3.0, 0.0, 0.6,  MAX_POWER, 300.0);
    drivePID.reset();
    turnPID.reset();

    std::uint32_t startMs = pros::millis();
    while (true) {
        update();

        double dx = targetX - xPos;
        double dy = targetY - yPos;
        double distanceError = std::sqrt(dx*dx + dy*dy);

        double angleToTarget = std::atan2(dy, dx);
        double headingError = angleToTarget - headingRad;
        while (headingError >  PI_VAL) headingError -= 2.0 * PI_VAL;
        while (headingError < -PI_VAL) headingError += 2.0 * PI_VAL;

        double dt = 0.02;
        double forward = drivePID.step(distanceError, dt);
        double turn = turnPID.step(headingError, dt);

        double leftPower  = forward - turn;
        double rightPower = forward + turn;

        // clamp
        if (leftPower  > MAX_POWER) leftPower  = MAX_POWER;
        if (leftPower  < -MAX_POWER) leftPower  = -MAX_POWER;
        if (rightPower > MAX_POWER) rightPower = MAX_POWER;
        if (rightPower < -MAX_POWER) rightPower = -MAX_POWER;

        setDrivePower(leftPower, rightPower);

        if (distanceError < 1.0) break;
        if ((pros::millis() - startMs) > maxTimeSeconds * 1000.0) break;

        pros::delay(20);
    }
    setDrivePower(0, 0);
}

//////////////////////////////////////////////////////////////////////
// Quadratic Bezier curve follow
//////////////////////////////////////////////////////////////////////
void Odom::curveTo(double x1, double y1,   // control (mid) point
                   double x2, double y2,   // final point
                   double maxTimeSeconds)
{
    PID speedPID(1.2, 0.0, 0.12, MAX_POWER, 600);
    PID turnPID (3.5, 0.0, 0.55, MAX_POWER, 200);
    speedPID.reset();
    turnPID.reset();

    std::uint32_t start = pros::millis();
    double x0 = xPos;  // start point
    double y0 = yPos;

    for (double t = 0.0; t <= 1.0; t += 0.005) {
        update();

        // Bezier curve point (quadratic)
        double bx = (1-t)*(1-t)*x0 + 2*(1-t)*t*x1 + t*t*x2;
        double by = (1-t)*(1-t)*y0 + 2*(1-t)*t*y1 + t*t*y2;

        // Derivative gives tangent (direction)
        double dxdt = -2*(1-t)*x0 + 2*(1-2*t)*x1 + 2*t*x2;
        double dydt = -2*(1-t)*y0 + 2*(1-2*t)*y1 + 2*t*y2;

        double targetHeading = std::atan2(dydt, dxdt);

        double distError = std::sqrt((bx - xPos)*(bx - xPos) + (by - yPos)*(by - yPos));
        double headingError = targetHeading - headingRad;
        while (headingError >  PI_VAL) headingError -= 2.0 * PI_VAL;
        while (headingError < -PI_VAL) headingError += 2.0 * PI_VAL;

        double dt = 0.02;
        double forward = speedPID.step(distError, dt);
        double turn    = turnPID.step(headingError, dt);

        double L = forward - turn;
        double R = forward + turn;

        // clamp
        if (L > MAX_POWER) L = MAX_POWER;
        if (L < -MAX_POWER) L = -MAX_POWER;
        if (R > MAX_POWER) R = MAX_POWER;
        if (R < -MAX_POWER) R = -MAX_POWER;

        setDrivePower(L, R);

        if ((pros::millis() - start) > maxTimeSeconds * 1000.0) break;

        pros::delay(20);
    }

    setDrivePower(0, 0);
}

void Odom::turnToHeading(double targetDeg, double maxTimeSeconds) {
    double targetRad = targetDeg * PI_VAL / 180.0;

    PID turnPID(4.0, 0.0, 0.5, MAX_POWER, 300.0);
    turnPID.reset();

    std::uint32_t start = pros::millis();
    while (true) {
        update();

        double error = targetRad - headingRad;
        while (error >  PI_VAL) error -= 2.0 * PI_VAL;
        while (error < -PI_VAL) error += 2.0 * PI_VAL;

        double power = turnPID.step(error, 0.02);

        setDrivePower(-power, power);

        if (std::fabs(error) < (2.0 * PI_VAL / 180.0)) break;  // ~2°
        if (pros::millis() - start > maxTimeSeconds * 1000.0) break;

        pros::delay(20);
    }
    setDrivePower(0, 0);
}

