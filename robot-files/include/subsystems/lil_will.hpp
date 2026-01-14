/**
 * @file lil_will.hpp
 * @brief Lil Will subsystem using pneumatics
 */

#ifndef LIL_WILL_HPP
#define LIL_WILL_HPP

#include "pros/adi.hpp"
#include "pros/misc.hpp"

class LilWill {
public:
  /**
   * @brief Constructor - initializes pneumatic solenoid
   */
  LilWill();

  /**
   * @brief Extend lil will mechanism
   */
  void extend();

  /**
   * @brief Retract lil will mechanism
   */
  void retract();

  /**
   * @brief Toggle lil will state
   */
  void toggle();

  /**
   * @brief Control lil will based on controller input
   * @param master Controller reference
   */
  void control(pros::Controller& master);

private:
  pros::adi::DigitalOut lilWillPneumatic;
  bool isExtended;
};

#endif // LIL_WILL_HPP