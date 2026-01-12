/**
 * @file drivetrain.cpp
 * @brief Implementation of the Drivetrain subsystem with LemLib integration
 *
 * This file contains the implementation for:
 * - Drivetrain construction with complete LemLib configuration
 * - Operator control driving with exponential curves
 * - Sensor calibration and initialization
 * - Real-time position telemetry display
 */

#include "subsystems/drivetrain.hpp"
#include "constants.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep∏
#include "lemlib/pose.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include <iostream>
#include <sstream>

#include "globals.hpp"

using namespace globals;

/**
 * @brief Drivetrain constructor - initializes all motors, sensors, and LemLib components
 *
 * Initialization order is critical due to dependencies:
 * 1. Motors and sensors (independent)
 * 2. Drive curves (independent)
 * 3. Tracking wheels (depend on rotation sensors)
 * 4. PID controllers (independent)
 * 5. Odometry sensor container (depends on tracking wheels + IMU)
 * 6. Drivetrain config (depends on motor groups)
 * 7. Chassis (depends on drivetrain, controllers, sensors, curves)
 */
Drivetrain::Drivetrain()
    : // Motor Groups - Blue cartridge (600 RPM) for balance of speed and torque
      leftMotorGroup(
          {PORT_VALUES::LEFT_1, PORT_VALUES::LEFT_2, PORT_VALUES::LEFT_3},
          pros::MotorGearset::blue), // 3 motors on left side (ports 20, 19, -18)
      rightMotorGroup(
          {PORT_VALUES::RIGHT_1, PORT_VALUES::RIGHT_2, PORT_VALUES::RIGHT_3},
          pros::MotorGearset::blue), // 3 motors on right side (ports -12, -13, 14)

      // Throttle Curve - Controls forward/backward joystick response
      throttleCurve(
          OPERATOR_CONSTANTS::THROTTLE::DEADBAND, // Deadband (0-127): Ignores small joystick drift near center
          OPERATOR_CONSTANTS::THROTTLE::MIN,      // Min output (0-127): Overcomes static friction
          OPERATOR_CONSTANTS::THROTTLE::CURVE     // Expo gain: >1.0 = more precise control at low speeds
          ),

      // Steer Curve - Controls turning joystick response
      steerCurve(
          OPERATOR_CONSTANTS::STEER::DEADBAND, // Deadband (0-127): Prevents unintended turning
          OPERATOR_CONSTANTS::STEER::MIN,      // Min output (0-127): Ensures responsive small turns
          OPERATOR_CONSTANTS::STEER::CURVE     // Expo gain: Allows precise aiming with quick turning capability
          ),

      // IMU Sensors - Dual IMU setup for sensor fusion (improved heading accuracy)
      imu1(PORT_VALUES::IMU_1),

      // Rotation Sensors for Tracking Wheels
      horizontalRotationSensor(PORT_VALUES::HORIZONTAL_ROTATION_SENSOR), // Port 6: Measures lateral movement
      verticalRotationSensor(PORT_VALUES::VERTICAL_ROTATION_SENSOR),     // Port 5: Measures forward/backward movement

      // Horizontal Tracking Wheel - Measures side-to-side (strafe) movement
      horizontalTrackingWheel(&horizontalRotationSensor,           // Pointer to rotation sensor
                              lemlib::Omniwheel::NEW_275,            // Wheel size: 2" diameter
                              CHASIS_VALUES::HORIZONTALTRACKING_WHEEL_OFFSET), // -3.966" offset from center

      // Vertical Tracking Wheel - Measures forward/backward movement
      verticalTrackingWheel(&verticalRotationSensor,              // Pointer to rotation sensor
                            lemlib::Omniwheel::NEW_275,             // Wheel size: 2" diameter
                            CHASIS_VALUES::LATERALTRACKING_WHEEL_OFFSET), // 0" offset (centered)
      // Lateral PID Controller - For forward/backward autonomous movement
      // Note: Currently all zeros (not yet tuned)
      lateralController(
          PID_CONSTANTS::LATERAL::KP,                   // Proportional gain: How hard to correct position error
          PID_CONSTANTS::LATERAL::KI,                   // Integral gain: Corrects accumulated error over time
          PID_CONSTANTS::LATERAL::KD,                   // Derivative gain: Dampens oscillation
          0,                                            // Anti-windup: Limits integral accumulation (0 = disabled)
          DRIVETRAIN_CONSTANTS::LATERAL::SMALL_ERROR,   // Small error threshold (inches): "Close enough" tolerance
          DRIVETRAIN_CONSTANTS::LATERAL::SMALL_TIMEOUT, // Small error timeout (ms): Time to stay in tolerance before settling
          DRIVETRAIN_CONSTANTS::LATERAL::LARGE_ERROR,   // Large error threshold (inches): Triggers timeout if exceeded
          DRIVETRAIN_CONSTANTS::LATERAL::LARGE_TIMEOUT, // Large error timeout (ms): Max time allowed in large error
          DRIVETRAIN_CONSTANTS::LATERAL::SLEW           // Slew rate (volts/sec): Max acceleration to prevent wheel slip
          ),

      // Angular PID Controller - For turning autonomous movement
      // Note: Tuned with KP=7.3, KD=6.5 for responsive turning
      angularController(
          PID_CONSTANTS::ANGULAR::KP,                   // Proportional gain: 7.3 (just above deadband for responsiveness)
          PID_CONSTANTS::ANGULAR::KI,                   // Integral gain: 0 (disabled to prevent wind-up)
          PID_CONSTANTS::ANGULAR::KD,                   // Derivative gain: 6.5 (reduced for small corrections)
          0,                                            // Anti-windup: Disabled
          DRIVETRAIN_CONSTANTS::ANGULAR::SMALL_ERROR,   // Small error threshold (degrees): Angular tolerance
          DRIVETRAIN_CONSTANTS::ANGULAR::SMALL_TIMEOUT, // Small error timeout (ms): Settlement time
          DRIVETRAIN_CONSTANTS::ANGULAR::LARGE_ERROR,   // Large error threshold (degrees): Error limit
          DRIVETRAIN_CONSTANTS::ANGULAR::LARGE_TIMEOUT, // Large error timeout (ms): Max time in error
          DRIVETRAIN_CONSTANTS::ANGULAR::SLEW           // Slew rate (deg/sec²): Max rotational acceleration
          ),

      // Odometry Sensor Container - Packages all sensors for LemLib
      // NOTE: LemLib v0.5.x only supports single IMU in constructor
      // imu2 is initialized as backup/redundancy but not used by LemLib automatic fusion
      sensors(&verticalTrackingWheel,   // Primary vertical wheel (forward/backward)
              nullptr,                  // Secondary vertical wheel (not used - nullptr)
              &horizontalTrackingWheel, // Primary horizontal wheel (lateral/strafe)
              nullptr,                  // Secondary horizontal wheel (not used - nullptr)
              &imu1                     // Primary IMU for heading tracking (imu2 available as backup)
              ),

      // Drivetrain Configuration - Physical robot specifications
      drivetrain(&leftMotorGroup,                // Pointer to left motor group (3 motors)
                 &rightMotorGroup,               // Pointer to right motor group (3 motors)
                 CHASIS_VALUES::TRACKWIDTH,      // Track width: 12 inches (left to right wheel distance)
                 lemlib::Omniwheel::NEW_325,     // Wheel type: 3.25" omni wheels
                 CHASIS_VALUES::RPM,             // Motor RPM: 450 (configured in constants)
                 CHASIS_VALUES::HORIZONTAL_DRIFT // Horizontal drift compensation: 8 inches
                 ),

      // Chassis Object - Main LemLib interface combining all components
      chassis(drivetrain,        // Drivetrain configuration (motors, dimensions)
              lateralController, // Lateral PID controller (forward/backward)
              angularController, // Angular PID controller (turning)
              sensors,           // Odometry sensors (tracking wheels + IMU)
              &throttleCurve,    // Throttle exponential curve (for driver control)
              &steerCurve        // Steer exponential curve (for driver control)
              ) {}               // Empty constructor body - all initialization in initializer list


/**
 * @brief Initialize and calibrate drivetrain sensors and motors
 *
 * This function performs crucial startup tasks:
 * 1. Calibrates IMU and resets tracking wheels to zero
 * 2. Sets motor brake modes for consistent stopping behavior
 * 3. Starts background telemetry task for position monitoring
 *
 * IMPORTANT: This blocks during IMU calibration (2-3 seconds).
 * Robot must be stationary during this time or calibration will fail.
 */
void Drivetrain::init() {
  chassis.calibrate();

  // Set motor brake modes to BRAKE for consistent stopping
  leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

  // Create background task for real-time position display on brain LCD
  // Lambda capture [&] allows access to chassis and imu1 members
  // pros::Task screenTask([&]() {
  //   pros::delay(10);
  //   while (true) {
  //     // Get current robot pose from odometry system
  //     lemlib::Pose pose = chassis.getPose();

  //     // Display position and status on brain LCD (5 lines)
  //     pros::lcd::print(0, "X: %f", pose.x);                    // X position (inches)
  //     pros::lcd::print(1, "Y: %f", pose.y);                    // Y position (inches)
  //     pros::lcd::print(2, "Theta: %f", pose.theta);            // Heading (degrees)
  //     pros::lcd::print(3, "IMU1: %f", imu1.get_heading()); // Primary IMU heading
  //     // pros::lcd::print(4, "IMU2: %f", imu2.get_heading()); // Secondary IMU heading
  //     pros::lcd::print(5, "vert rot: %i", verticalRotationSensor.get_position()); // Secondary IMU heading
  //     pros::lcd::print(6, "hortizontal rot: %i", horizontalRotationSensor.get_position()); // Secondary IMU heading

  //     // Log position data to LemLib telemetry system (for graphing/debugging)
  //     lemlib::telemetrySink()->info("Chassis pose: x: %f, y: %f, theta: %f",
  //                                   pose.x, pose.y, pose.theta);

  //     // Update every 50ms (20Hz refresh rate) to balance responsiveness and CPU usage
  //     pros::delay(50);
  //   }
  // });
}

/**
 * @brief Operator control drive function - implements arcade drive with special turning logic
 *
 * Called every iteration of the opcontrol loop (every 15ms).
 *
 * Drive scheme:
 * - Arcade drive: Left stick Y = forward/backward, Right stick X = turning
 * - Applies exponential curves for fine control (configured in OPERATOR_CONSTANTS)
 * - Special case: When throttle is near zero, applies steer curve for precise turning
 *
 * The special turning logic allows for:
 * - Precise point-turning when not moving forward
 * - Smooth arcing turns when moving (handled automatically by chassis.arcade)
 */
void Drivetrain::drive() {
  // Read joystick values from controller (-127 to +127)
  int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);  // Forward/backward
  int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);     // Left/right turn

  chassis.arcade(-throttle, -turn, false, OPERATOR_CONSTANTS::DESATURATE_BIAS);
}

/**
 * @brief Get reference to the LemLib chassis object for autonomous control
 * @return Reference to the internal chassis object
 *
 * Use this accessor in autonomous routines to call LemLib movement functions:
 * - chassis.moveToPoint(x, y, timeout) - Move to field coordinates
 * - chassis.turnToHeading(angle, timeout) - Turn to absolute heading
 * - chassis.turnToPoint(x, y, timeout) - Face towards a point
 * - chassis.follow(path, timeout, lookahead) - Follow a motion profile path
 * - chassis.setPose(x, y, heading) - Set current position (for field-relative autonomous)
 */
lemlib::Chassis& Drivetrain::get_chassis() {
  return chassis;
}

/**
 * @brief Get left motor group for debugging
 * @return Reference to left motor group
 */
pros::MotorGroup& Drivetrain::get_left_motors() {
  return leftMotorGroup;
}

/**
 * @brief Get right motor group for debugging
 * @return Reference to right motor group
 */
pros::MotorGroup& Drivetrain::get_right_motors() {
  return rightMotorGroup;
}
