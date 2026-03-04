#include "subsystems/drivetrain.hpp"
#include <cmath>

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
      verticalTrackingWheel(&verticalRotationSensor,
                            lemlib::Omniwheel::NEW_275,
                            CHASIS_VALUES::VERTICAL_TRACKING_WHEEL_OFFSET
                        ),
      sensors(&verticalTrackingWheel, nullptr, nullptr, nullptr, &imu1),
      drivetrain(&leftMotorGroup,
                 &rightMotorGroup,
                 CHASIS_VALUES::TRACKWIDTH,
                 lemlib::Omniwheel::NEW_4,
                 CHASIS_VALUES::RPM,
                 CHASIS_VALUES::HORIZONTAL_DRIFT
                ),
      chassis(drivetrain, lateralController, angularController, sensors) {}

void Drivetrain::init() {
    // Reverse tracking wheel so forward motion reads positive
    verticalRotationSensor.set_reversed(true);

    // Set motor brake modes
    leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // Calibrate the chassis (IMU and odometry)
    chassis.calibrate();
}

void Drivetrain::drive() {
    // Use the shared global controller for input
    auto& master = globals::controller;

    // Get joystick values DO NOT CHANGE THIS SHIT IT WORKS IT WORKS IT WORKS DO NOT TOUCH PLEASE PLEASE PLEASE PLEASE
    const int rawTurn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); //DO NOT TOUCH
    const int rawThrottle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); //DO NOT TOUCH

    // Apply expo drive curves using configured deadband/min/curve constants
    const int throttle = throttleCurve.curve(rawThrottle);
    const int turn = steerCurve.curve(rawTurn);

    // Snap to nearest N/E/S/W (Y button)
    if (master.get_digital_new_press(CONTROLLER_BUTTONS::DRIVETRAIN::SNAP_CARDINAL)) {
        snapToCardinal();
        return;
    }

    // Don't let manual arcade drive override an in-progress snap motion
    if (chassis.isInMotion()) return;

    // Arcade drive using lemlib, with desaturation bias from constants
    chassis.arcade(throttle,
                   -turn,
                   true, // built-in drive curve disabled; we apply our own above
                   OPERATOR_CONSTANTS::DESATURATE_BIAS);
}

void Drivetrain::navigateTo(const Waypoint& wp) {
    // moveToPose moves to (x, y) and arrives at the given heading
    // async=false blocks until motion completes or timeout fires
    chassis.moveToPose(wp.x, wp.y, wp.heading, wp.timeout, {}, false);
}

void Drivetrain::snapToCardinal() {
    // Snap to nearest 0 (N), 90 (E), 180 (S), or 270 (W)
    float heading = std::fmod(std::fmod(chassis.getPose().theta, 360.0f) + 360.0f, 360.0f);
    float target = std::round(heading / 90.0f) * 90.0f;
    if (target >= 360.0f) target = 0.0f;
    chassis.turnToHeading(target, DRIVETRAIN_CONSTANTS::SNAP::TIMEOUT, {}, true); // async=true; isInMotion() guard blocks arcade
}

void Drivetrain::run() {
    // Main run method to be called in the robot loop
    drive();

    // Display current pose on the V5 brain screen
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %.2f  Y: %.2f", pose.x, pose.y);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Angle: %.2f deg", pose.theta);
}

lemlib::Chassis& Drivetrain::get_chassis() { return chassis; }

pros::MotorGroup& Drivetrain::get_left_motors() { return leftMotorGroup; }

pros::MotorGroup& Drivetrain::get_right_motors() { return rightMotorGroup; }
