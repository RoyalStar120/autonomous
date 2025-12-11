#include "pid.hpp"
#include <algorithm>

PID::PID(double p, double i, double d, double output_limit, double integral_limit)
    : kP(p), kI(i), kD(d),
      integral(0.0), prevError(0.0),
      outputLimit(output_limit), integralLimit(integral_limit) {}

void PID::reset() {
    integral = 0.0;
    prevError = 0.0;
}

double PID::step(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    integral += error * dt;
    if (integralLimit > 0.0) {
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;
    }

    double derivative = (error - prevError) / dt;
    double out = kP * error + kI * integral + kD * derivative;

    if (outputLimit > 0.0) {
        if (out > outputLimit) out = outputLimit;
        if (out < -outputLimit) out = -outputLimit;
    }

    prevError = error;
    return out;
}
