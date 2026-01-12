/**
 * @file globals.cpp
 * @brief Global variable instantiation and initialization
 *
 * This file contains the actual definitions and memory allocations for global
 * variables declared in globals.hpp. These variables are accessible throughout
 * the entire program and maintain shared state across subsystems.
 *
 * Design note: While global variables are generally discouraged in larger
 * applications, they are commonly used in VEX robotics code for:
 * - Sharing the controller object across subsystems
 * - Maintaining match state (alliance color, autonomous selection)
 * - Simplifying code in resource-constrained embedded environments
 */

#include "globals.hpp"
#include "constants.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/vision.h"

namespace globals {

/**
 * @brief Master controller instantiation
 *
 * Creates the primary controller object used by all subsystems to read driver
 * input. Initialized as the master controller (as opposed to partner
 * controller).
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * @brief Default alliance color
 *
 * This determines which color game pieces
 * to keep vs auto-eject during matches.
 */
SensorColors alliance = SensorColors::RED;
} // namespace globals