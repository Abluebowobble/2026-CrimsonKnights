#include "subsystems/drivetrain.hpp"

// Constructor: configure motors, sensors, controller settings, and lemlib chassis
Drivetrain::Drivetrain(): 

    leftMotorGroup({
                    PORT_VALUES::LEFT_1,
                    PORT_VALUES::LEFT_2,
                    PORT_VALUES::LEFT_3},
                    pros::MotorGears::blue
                ),
      rightMotorGroup({
                    PORT_VALUES::RIGHT_1, 
                    PORT_VALUES::RIGHT_2, 
                    PORT_VALUES::RIGHT_3},
                    pros::MotorGears::blue
                ),
      imu1(PORT_VALUES::IMU_1),

      throttleCurve(
                    OPERATOR_CONSTANTS::THROTTLE::DEADBAND,
                    OPERATOR_CONSTANTS::THROTTLE::MIN,
                    OPERATOR_CONSTANTS::THROTTLE::CURVE
                    ),
      steerCurve(OPERATOR_CONSTANTS::STEER::DEADBAND,
                 OPERATOR_CONSTANTS::STEER::MIN,
                 OPERATOR_CONSTANTS::STEER::CURVE
                ),

      lateralController(DRIVETRAIN_CONSTANTS::LATERAL::KP,
                        DRIVETRAIN_CONSTANTS::LATERAL::KI,
                        DRIVETRAIN_CONSTANTS::LATERAL::KD,
                        DRIVETRAIN_CONSTANTS::LATERAL::ANTI_WINDUP_RANGE,
                        DRIVETRAIN_CONSTANTS::LATERAL::SMALL_ERROR,
                        DRIVETRAIN_CONSTANTS::LATERAL::SMALL_TIMEOUT,
                        DRIVETRAIN_CONSTANTS::LATERAL::LARGE_ERROR,
                        DRIVETRAIN_CONSTANTS::LATERAL::LARGE_TIMEOUT,
                        DRIVETRAIN_CONSTANTS::LATERAL::SLEW
                    ),
      angularController(DRIVETRAIN_CONSTANTS::ANGULAR::KP,
                        DRIVETRAIN_CONSTANTS::ANGULAR::KI,
                        DRIVETRAIN_CONSTANTS::ANGULAR::KD,
                        DRIVETRAIN_CONSTANTS::ANGULAR::ANTI_WINDUP_RANGE,
                        DRIVETRAIN_CONSTANTS::ANGULAR::SMALL_ERROR,
                        DRIVETRAIN_CONSTANTS::ANGULAR::SMALL_TIMEOUT,
                        DRIVETRAIN_CONSTANTS::ANGULAR::LARGE_ERROR,
                        DRIVETRAIN_CONSTANTS::ANGULAR::LARGE_TIMEOUT,
                        DRIVETRAIN_CONSTANTS::ANGULAR::SLEW
                    ),
      verticalRotationSensor(PORT_VALUES::VERTICAL_ROTATION_SENSOR),
      horizontalRotationSensor(PORT_VALUES::HORIZONTAL_ROTATION_SENSOR),
      verticalTrackingWheel(&verticalRotationSensor,
                            lemlib::Omniwheel::NEW_275,
                            CHASIS_VALUES::LATERALTRACKING_WHEEL_OFFSET
                        ),
      horizontalTrackingWheel(&horizontalRotationSensor,
                              lemlib::Omniwheel::NEW_275,
                              CHASIS_VALUES::HORIZONTALTRACKING_WHEEL_OFFSET
                            ),
      sensors(nullptr, nullptr, nullptr, nullptr, nullptr),
      drivetrain(&leftMotorGroup,
                 &rightMotorGroup,
                 CHASIS_VALUES::TRACKWIDTH,
                 lemlib::Omniwheel::NEW_4,
                 CHASIS_VALUES::RPM,
                 CHASIS_VALUES::HORIZONTAL_DRIFT
                ),
      chassis(drivetrain, lateralController, angularController, sensors) {}

void Drivetrain::init() {
    // Set motor brake modes
    leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Calibrate the chassis (IMU and odometry)
    chassis.calibrate();
}

void Drivetrain::drive() {
    // Use the shared global controller for input
    auto& master = globals::controller;

    // Get joystick values
    const int rawThrottle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int rawTurn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Arcade drive using lemlib, with desaturation bias from constants
    chassis.arcade(rawThrottle,
                   -rawTurn,
                   false, // disable built-in drive curve since we apply our own
                   OPERATOR_CONSTANTS::DESATURATE_BIAS);
}

void Drivetrain::run() {
    // Main run method to be called in the robot loop
    drive();
}

lemlib::Chassis& Drivetrain::get_chassis() { return chassis; }

pros::MotorGroup& Drivetrain::get_left_motors() { return leftMotorGroup; }

pros::MotorGroup& Drivetrain::get_right_motors() { return rightMotorGroup; }
