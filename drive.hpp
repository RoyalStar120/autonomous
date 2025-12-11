#ifndef DRIVE_HPP
#define DRIVE_HPP

class Drive {
public:
    Drive() = default;
    // left/right in [-127,127]
    void setPower(double left, double right);
    void stop();
};

#endif // DRIVE_HPP
