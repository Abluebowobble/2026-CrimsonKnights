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
    // Run motor at high speed for scoring high
    spin(127);
  
}

void EndEffector::scoreMid() {
    // Run motor at medium speed for scoring mid
    spin(90);
}

void EndEffector::control(pros::Controller& master) {
    if (master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_HIGH)) {
        scoreHigh();
    } else if (master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_MID)) {
        scoreMid();
    } else if (master.get_digital(CONTROLLER_BUTTONS::E_CONTROLLER_DIGITAL_R2)) {
        spin(-127);
    }
    else {
        stop();
    }

}

void EndEffector::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}