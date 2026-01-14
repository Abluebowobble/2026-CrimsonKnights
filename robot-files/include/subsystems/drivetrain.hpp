/**
 * @file drivetrain.hpp
 * @brief Drivetrain subsystem class definition with 6-motor holonomic drive and odometry
 *
 * This subsystem manages:
 * - 6 VEX V5 motors (3 left, 3 right) for differential drive
 * - LemLib chassis integration for autonomous navigation
 * - Odometry tracking using 2 tracking wheels + IMU sensor
 * - Exponential drive curves for smooth operator control
 * - Real-time position display on brain screen
 */

#ifndef DRIVETRAINSUBSYTEMS_H
#define DRIVETRAINSUBSYTEMS_H

#include "lemlib/api.hpp" // for lemlib::Chassis, ExpoDriveCurve, ControllerSettings, TrackingWheel
#include "constants.hpp"   // for port and tuning constants
#include "globals.hpp"     // for globals::controller

/**
 * @class Drivetrain
 * @brief Manages robot movement, odometry, and chassis control
 *
 * Architecture:
 * - Uses LemLib for advanced motion algorithms (PID, pure pursuit, odometry)
 * - Implements arcade drive control (single stick forward/back, other stick turn)
 * - Tracks robot position on field using sensor fusion (tracking wheels + IMU)
 * - Applies exponential curves to joystick inputs for fine control
 */
class Drivetrain {
public:
  /**
   * @brief Constructor - initializes all motors, sensors, and LemLib components
   *
   * Sets up:
   * - Motor groups with correct polarities from PORT_VALUES
   * - IMU sensor(s) for heading tracking
   * - Tracking wheels for position tracking
   * - Exponential drive curves for joystick smoothing
   * - PID controllers for autonomous movement
   * - LemLib chassis object with all configured components
   */
  Drivetrain();

  /**
   * @brief Operator control drive method - call every loop iteration
   *
   * Reads controller joysticks and applies arcade drive control:
   * - Left stick Y-axis: Forward/backward throttle
   * - Right stick X-axis: Turning/rotation
   * - Applies exponential curves for smooth control
   * - Special handling: Enhanced turning sensitivity when throttle is near zero
   */
  void drive();

  /**
   * @brief Main run method - call this in the robot loop
   *
   * This method handles all drivetrain operations during teleoperated mode.
   * It calls drive() and can be extended to include additional functionality.
   */
  void run();

  /**
   * @brief Initialize drivetrain - calibrates sensors and starts telemetry
   *
   * Performs:
   * - Sensor calibration (IMU, tracking wheels)
   * - Sets motor brake modes to BRAKE (coast would be E_MOTOR_BRAKE_COAST)
   * - Starts background task for LCD position display
   * - Prepares chassis for operation
   *
   * Call this during initialize() phase before competition starts
   */
  void init();

  /**
   * @brief Accessor for LemLib chassis object
   * @return Reference to the internal chassis object for autonomous control
   *
   * Use this to access LemLib movement functions like:
   * - chassis.moveToPoint(x, y, timeout)
   * - chassis.turnToHeading(angle, timeout)
   * - chassis.setPose(x, y, heading)
   */
  lemlib::Chassis& get_chassis();

  /**
   * @brief Get left motor group for debugging
   * @return Reference to left motor group
   */
  pros::MotorGroup& get_left_motors();

  /**
   * @brief Get right motor group for debugging
   * @return Reference to right motor group
   */
  pros::MotorGroup& get_right_motors();

private:
  // ====================
  // MOTORS
  // ====================
  pros::MotorGroup leftMotorGroup;  ///< Left side motor group (ports from PORT_VALUES::LEFT_* constants)
  pros::MotorGroup rightMotorGroup; ///< Right side motor group (ports from PORT_VALUES::RIGHT_* constants)

  // ====================
  // SENSORS
  // ====================
  pros::Imu imu1; ///< Primary IMU for heading tracking (PORT_VALUES::IMU_1)
  // pros::Imu imu2; ///< Secondary IMU for sensor fusion (more accurate heading)

  // ====================
  // DRIVE CURVES
  // ====================
  lemlib::ExpoDriveCurve throttleCurve; ///< Exponential curve for forward/backward control (OPERATOR_CONSTANTS::THROTTLE)
  lemlib::ExpoDriveCurve steerCurve;    ///< Exponential curve for turning control (OPERATOR_CONSTANTS::STEER)

  // ====================
  // PID CONTROLLERS
  // ====================
  lemlib::ControllerSettings lateralController; ///< PID settings for forward/backward autonomous movement (DRIVETRAIN_CONSTANTS::LATERAL)
  lemlib::ControllerSettings angularController; ///< PID settings for rotational autonomous movement (DRIVETRAIN_CONSTANTS::ANGULAR)

  // ====================
  // ODOMETRY SENSORS
  // ====================
  pros::Rotation verticalRotationSensor;   ///< Rotation sensor for vertical tracking wheel (forward/back)
  pros::Rotation horizontalRotationSensor; ///< Rotation sensor for horizontal tracking wheel (strafe/lateral)

  lemlib::TrackingWheel verticalTrackingWheel;   ///< Vertical tracking wheel object (measures forward/back)
  lemlib::TrackingWheel horizontalTrackingWheel; ///< Horizontal tracking wheel object (measures lateral movement)

  // ====================
  // LEMLIB COMPONENTS
  // ====================
  lemlib::OdomSensors sensors;     ///< Container for all odometry sensors (tracking wheels + IMU)
  lemlib::Drivetrain drivetrain;   ///< LemLib drivetrain configuration (motors, dimensions, wheel size)
  lemlib::Chassis chassis;         ///< Main LemLib chassis object - handles movement and odometry
};

#endif

