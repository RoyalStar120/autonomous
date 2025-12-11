#include "intake.hpp"

Intake::Intake()
    : bottomIntake(PORT_INTAKE_BOTTOM),
      topIntake(PORT_INTAKE_TOP),
      roller(PORT_ROLLER) {}

// ===== INTAKE CONTROLS =====

void Intake::in(int power) {
    bottomIntake.move(power);
    topIntake.move(power);
}

void Intake::out(int power) {
    bottomIntake.move(-power);
    topIntake.move(-power);
}

void Intake::stop() {
    bottomIntake.move(0);
    topIntake.move(0);
}

// ===== ROLLER CONTROLS =====

void Intake::rollerIn(int power) {
    roller.move(power);
}

void Intake::rollerOut(int power) {
    roller.move(-power);
}

void Intake::rollerStop() {
    roller.move(0);
}
