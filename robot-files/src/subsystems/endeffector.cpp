#include "subsystems/endeffector.hpp"
#include "constants.hpp"
#include "globals.hpp"
#include "pros/misc.hpp"

EndEffector::EndEffector()
    : endEffectorMotor(PORT_VALUES::ENDEFFECTOR_MOTOR_PORT, pros::MotorGears::blue),
      isScoring(false) {
    endEffectorMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void EndEffector::spin(int velocity) {
    endEffectorMotor.move(velocity);
}

void EndEffector::stop() {
    endEffectorMotor.move(0);
}

void EndEffector::scoreHigh() {
    spin(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_HIGH_SPEED);
}

void EndEffector::scoreMid() {
    spin(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_MID_SPEED);
}

void EndEffector::control(pros::Controller& master) {
    if (master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_HIGH)) {
        scoreHigh();
    } else if (master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_MID)) {
        scoreMid();
    }
    else {
        stop();
    }

}

void EndEffector::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}