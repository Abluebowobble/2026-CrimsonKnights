/**
 * @file globals.hpp
 * @brief Global variables and shared data structures accessible throughout the entire program
 *
 * This file declares global objects and enumerations that need to be shared across multiple
 * subsystems and source files. The actual variable definitions are in globals.cpp.
 */

#pragma once

#include "api.h" // for pros::Controller

/**
 * @namespace globals
 * @brief Contains all globally accessible variables and shared state
 */
namespace globals {

/**
 * @brief Master controller object for driver input
 *
 * This controller is shared across all subsystems to read button states and joystick values.
 * Defined as extern here, instantiated in globals.cpp.
 */
extern pros::Controller controller;

/**
 * @enum SensorColors
 * @brief Represents detected alliance colors from optical sensors
 *
 * Used for:
 * - Identifying game piece colors (red vs blue alliance)
 * - Auto-ejecting opponent alliance game pieces
 * - Setting the robot's current alliance color for scoring logic
 */
enum class SensorColors {
    BLUE, ///< Blue alliance color detected (hue range 90-260)
    RED,  ///< Red alliance color detected (hue range 340-360 or 0-10)
    NONE  ///< No valid color detected or sensor not seeing a game piece
};

/**
 * @brief Current alliance color set during match initialization
 *
 * This value is set by the autonomous selector screen and determines:
 * - Which color game pieces to keep vs eject
 * - Autonomous routine selection (red vs blue side routines)
 *
 * Defined as extern here, instantiated in globals.cpp with default value.
 */
extern SensorColors alliance;

} // namespace globals