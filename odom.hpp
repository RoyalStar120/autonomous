#ifndef ODOM_HPP
#define ODOM_HPP

#include <cstdint>

// These MUST exist in config.hpp:
// driveLF, driveLM, driveLB, driveRF, driveRM, driveRB
// INCHES_PER_DEGREE
// TRACK_WIDTH_INCHES
// MAX_POWER
#include "config.hpp"
#include "pid.hpp"

class Odom {
public:
    Odom();

    // Reset robot pose (x, y in inches, heading in degrees)
    void reset(double x, double y, double heading_deg);

    // Odometry update (call every 10–20ms)
    void update();

    // Drive to a coordinate using PID (inches + degrees)
    void driveTo(double targetX, double targetY,
                 double targetHeadingDeg,
                 double maxTimeSeconds);

    // Bezier curve driving
    void curveTo(double x1, double y1,
                 double x2, double y2,
                 double maxTimeSeconds);

    // Turn-to-heading using PID
    void turnToHeading(double targetDeg,
                       double maxTimeSeconds);

    // Public pose
    double xPos;
    double yPos;
    double headingRad;

private:
    // Odometry helpers
    double getLeftDeg() const;
    double getRightDeg() const;

    double lastLeftDeg;
    double lastRightDeg;
    std::uint32_t lastUpdateMs;
};

#endif // ODOM_HPP
