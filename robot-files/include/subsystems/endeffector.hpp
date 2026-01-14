/**
 * @file endeffector.hpp
 * @brief End effector subsystem for scoring game elements
 */

#ifndef ENDEFFECTOR_HPP
#define ENDEFFECTOR_HPP

#include "pros/motors.hpp"
#include "pros/misc.hpp"

class EndEffector {
public:
  /**
   * @brief Constructor - initializes end effector motor
   */
  EndEffector();

  /**
   * @brief Runs end effector motor
   * @param velocity Motor velocity (-127 to 127)
   */
  void spin(int velocity);

  /**
   * @brief Stops the end effector
   */
  void stop();

  /**
   * @brief Score at high position
   */
  void scoreHigh();

  /**
   * @brief Score at mid position
   */
  void scoreMid();

  /**
   * @brief Control end effector based on controller input
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
  pros::Motor endEffectorMotor;
  bool isScoring;
};

#endif // ENDEFFECTOR_HPP