#include "auton.hpp"

// Define global odom instance
Odom odom;

void runMatchAuton() {
    // Example: start at origin, face +X (0 degrees)
    odom.reset(-135.407, 33.835, 90);
    
    odom.curveTo(
    -126,0,44.0   // Mid control point
    -127.659, 85.829,   // End of curve
    1.6                 // More time for a smoother curve
    );

    odom.driveTo(-77.285, 67.128, 123.1, 2.0);
    odom.turnToHeading(90, 1);
} 

void runSkillsAuton() {
    odom.reset(0.0, 0.0, 0.0);

    // add longer skills path here
}

void opcontrol() {
    while (true) {
        pros::delay(20);
    }
}
    
