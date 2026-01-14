#include "subsystems/endeffector.hpp"
#include "constants.hpp"
#include "globals.hpp"
#include "pros/misc.hpp"

EndEffector::EndEffector()
    : endEffectorMotor(PORT_VALUES::ENDEFFECTOR_MOTOR_PORT, pros::MotorGears::green),
      isScoring(false) {
    endEffectorMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void EndEffector::spin(int velocity) {
    endEffectorMotor.move(velocity);
}

void EndEffector::stop() {
    endEffectorMotor.move(0);
    isScoring = false;
}

void EndEffector::scoreHigh() {
    // Run motor at high speed for scoring high
    spin(127);
    isScoring = true;
}

void EndEffector::scoreMid() {
    // Run motor at medium speed for scoring mid
    spin(90);
    isScoring = true;
}

void EndEffector::control(pros::Controller& master) {
    if (master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_HIGH)) {
        scoreHigh();
    } else if (master.get_digital(CONTROLLER_BUTTONS::ENDEFFECTOR::SCORE_MID)) {
        scoreMid();
    } else if (!isScoring) {
        stop();
    }
    
    // Auto-stop after scoring completes (can be refined with timers/sensors)
    if (isScoring && endEffectorMotor.get_actual_velocity() < 5) {
        stop();
    }
}

void EndEffector::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}