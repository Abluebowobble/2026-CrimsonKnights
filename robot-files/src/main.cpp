#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "subsystems/drivetrain.hpp"
#include "subsystems/little_will.hpp"
#include "subsystems/endeffector.hpp"
#include "subsystems/intake.hpp"
// #include "autonomous/waypoint.hpp"      // shelved: modular auton system
// #include "autonomous/auton_sequence.hpp" // shelved: modular auton system


Drivetrain drivetrain;
Intake intake;

EndEffector endeffector;
LittleWill littlewill;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  pros::lcd::register_btn1_cb(on_center_button);
  drivetrain.init();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  auto& chassis = drivetrain.get_chassis();

  // Background task to print pose to brain screen during auton
  pros::Task screenTask([&]() {
    while (true) {
      lemlib::Pose pose = chassis.getPose();
      pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %.2f  Y: %.2f", pose.x, pose.y);
      pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Angle: %.2f deg", pose.theta);
      pros::delay(50);
    }
  });

  chassis.setPose(0, 0, 0);

  intake.spin(-127);
  //3stack
  chassis.moveToPose(10, 13, 45, 3000, {.maxSpeed = 60}, false);
  chassis.moveToPose(25, 29, 45, 3000, {.maxSpeed = 20}, false);
  littlewill.extend();
  pros::delay(800);
  //under goal
  // chassis.moveToPose(32, 40, 65, 3000, {.maxSpeed = 40}, false);
  

  littlewill.extend();
  pros::delay(800);
  //move back
  chassis.moveToPose(20, 20, 45, 3000, {.forwards=false,.maxSpeed = 100}, false);
  //mid spot
  chassis.moveToPose(44, 0, 180, 3000, {.maxSpeed = 80}, false);
  //goal
  chassis.moveToPose(44, 24, 180, 1500, {.forwards = false, .maxSpeed = 70}, false);
  endeffector.scoreHigh();
  pros::delay(2000);
  endeffector.stop();
  //match loader
  littlewill.extend();
  pros::delay(400);
  chassis.moveToPose(44, -6, 0, 4000, {.maxSpeed = 40}, true);
  pros::delay(2500);
  //goal
  chassis.moveToPose(44, 24, 180, 3000, {.forwards = false, .maxSpeed = 70}, false);
  

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  while (true) {
    // B button: reset odometry position to (0, 0, 0)
    if (globals::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      drivetrain.get_chassis().setPose(0, 0, 0);
    }

    // Right arrow: run autonomous routine
    if (globals::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      autonomous();
    }

    // Run drivetrain subsystem
    drivetrain.run();
    intake.run();
    littlewill.run();
    endeffector.run();
    // wing.run();
    
    // Small delay to prevent CPU overuse
    pros::delay(10);
  }
}
