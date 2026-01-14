/**
 * @file wing.hpp
 * @brief Wing subsystem using pneumatics
 */

#ifndef WING_HPP
#define WING_HPP

#include "pros/adi.hpp"
#include "pros/misc.hpp"

class Wing {
public:
  /**
   * @brief Constructor - initializes pneumatic solenoid
   */
  Wing();

  /**
   * @brief Extend the wing
   */
  void extend();

  /**
   * @brief Retract the wing
   */
  void retract();

  /**
   * @brief Toggle wing state
   */
  void toggle();

  /**
   * @brief Control wing based on controller input
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
  pros::adi::DigitalOut wingPneumatic;
  bool isExtended;
};

#endif // WING_HPP