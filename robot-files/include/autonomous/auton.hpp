#pragma once

#include <string>

class Autonomous {
public:
  enum AUTON_ROUTINE {
    RED_NEG = 1,
    RED_POS = 2,
    RED_POS_LATE_RUSH = 3,
    BLUE_POS = -1,
    BLUE_POS_LATE_RUSH = -2,
    BLUE_NEG = -3,
    SKILLS = 0
  };

  /**
   * @brief Sets the number of the autonomous program to use.
   *
   * This function allows the user to specify the autonomous program to be
   * executed by the robot. The autonomous program number determines the
   * specific actions and movements the robot will perform.
   *
   * @param auton The number of the autonomous program to use.
   */
  static AUTON_ROUTINE auton;

  /**
   * @brief The name of the autonomous program.
   * @details This variable stores the name of the autonomous program currently
   * selected. It is handled by the switching functions and the screen.
   */
  static std::string autonName;

  /**
   * @brief Drives the robot autonomously.
   *
   * This function drives the robot autonomously based on the selected
   * autonomous program. It takes a reference to a Puncher object and a boolean
   * value indicating whether to use autonomous mode.
   *
   * @param puncher A reference to the Puncher object.
   * @param autono A boolean value indicating whether to use autonomous mode.
   */
  void AutoDrive();

  /**
   * @brief Switches the autonomous program.
   *
   * This function switches the autonomous program to the next available
   * program. It allows the user to cycle through different autonomous programs
   * during runtime.
   */
  static void AutonSwitcher(int autonNum);
};

// Autonomous routines
void right_side_auto();
void left_side_auto();