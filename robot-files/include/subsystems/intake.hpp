/**
 * @file intake.hpp
 * @brief Intake subsystem for collecting game elements
 */

#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "pros/motors.hpp"
#include "pros/misc.hpp"

class Intake {
public:
  /**
   * @brief Constructor - initializes intake motor
   */
  Intake();

  /**
   * @brief Runs intake to collect game elements
   * @param velocity Motor velocity (-127 to 127)
   */
  void spin(int velocity);

  /**
   * @brief Stops the intake
   */
  void stop();

  /**
   * @brief Control intake based on controller input
   * @param master Controller reference
   */
  void control(pros::Controller& master);

  /**
   * @brief Run method to be called from the main robot loop.
   *
   * Internally uses the global controller and links buttons to actions.
   */
  void run();

private:
  pros::Motor intakeMotor;
};

#endif // INTAKE_HPP