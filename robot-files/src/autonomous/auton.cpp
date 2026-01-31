#include "autonomous/auton.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/endeffector.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/lil_will.hpp"
#include "subsystems/wing.hpp"
#include <iostream>
#include <string>

using namespace globals;

// External reference to subsystems
extern struct RobotSubsystems {
  Drivetrain drivetrain;
  EndEffector endeffector;
  Intake intake;
  LilWill lilWill;
  Wing wing;
} subsystems;

Autonomous::AUTON_ROUTINE Autonomous::auton = RED_POS;
std::string Autonomous::autonName = "Red Positive";

constexpr int delay_constant = 1050;

void test() {
  subsystems.lilWill.extend();
  pros::delay(1000);
  subsystems.lilWill.retract();
  pros::delay(1000);

  subsystems.wing.extend();
  pros::delay(1000);
  subsystems.wing.retract();
}