#include "drive.hpp"
#include "config.hpp"

void Drive::setPower(double left, double right) {
    setDrivePower(left, right);
}

void Drive::stop() {
    setDrivePower(0, 0);
}
