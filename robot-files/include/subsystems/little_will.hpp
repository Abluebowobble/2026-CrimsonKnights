/**
 * @file little_will.hpp
 * @brief Little Will subsystem using pneumatics
 */

#ifndef LITTLE_WILL_HPP
#define LITTLE_WILL_HPP

#include "pros/adi.hpp"
#include "pros/misc.hpp"

class LittleWill {
public:
  /**
   * @brief Constructor - initializes pneumatic solenoid
   */
  LittleWill();

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

  /**
   * @brief Run method to be called from the main robot loop.
   *
   * Internally uses the global controller and links buttons to actions.
   */
  void run();

private:
  pros::adi::Pneumatics littleWillPneumatic;
  bool isExtended;
};

#endif // LITTLE_WILL_HPP