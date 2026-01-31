#include "subsystems/intake.hpp"
#include "constants.hpp"
#include "globals.hpp"
#include "pros/misc.hpp"

Intake::Intake()
    : intakeMotor(PORT_VALUES::INTAKE_MOTOR_PORT, pros::MotorGears::green) {
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Intake::spin(int velocity) {
    intakeMotor.move(velocity);
}

void Intake::stop() {
    intakeMotor.move(0);
}

void Intake::control(pros::Controller& master) {
    if (master.get_digital(CONTROLLER_BUTTONS::INTAKE::INTAKE)||master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_HIGH)
||master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_MID)) {
        spin(-127); // Full speed intake
    } else if (master.get_digital(CONTROLLER_BUTTONS::INTAKE::OUTTAKE)) {
        spin(127); // Full speed outtake
    } else {
        stop();
    }
}

void Intake::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}