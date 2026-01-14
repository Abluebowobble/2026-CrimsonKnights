#include "subsystems/drivetrain.hpp"
#include "pros/misc.hpp"
#include "pros/screen.hpp"

// Constructor
Drivetrain::Drivetrain()
    : leftMotorGroup({20, 19, -18}, pros::MotorGears::blue),
      rightMotorGroup({-12, -13, 14}, pros::MotorGears::blue),
      imu1(10),
      throttleCurve(3, 10, 1.019),
      steerCurve(3, 10, 1.019),
      lateralController(10, 0, 3, 3, 1, 100, 3, 500, 20),
      angularController(2, 0, 10, 3, 1, 100, 3, 500, 0),
      verticalRotationSensor(11),
      horizontalRotationSensor(20),
      verticalTrackingWheel(&verticalRotationSensor, lemlib::Omniwheel::NEW_275, -2.5),
      horizontalTrackingWheel(&horizontalRotationSensor, lemlib::Omniwheel::NEW_275, -5.75),
      sensors(&verticalTrackingWheel, nullptr, &horizontalTrackingWheel, nullptr, &imu1),
      drivetrain(&leftMotorGroup, &rightMotorGroup, 10, lemlib::Omniwheel::NEW_4, 360, 2),
      chassis(drivetrain, lateralController, angularController, sensors) {}

void Drivetrain::init() {
    // Set motor brake modes
    leftMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotorGroup.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    
    // Calibrate the chassis
    chassis.calibrate();
    
    // Start background task for position display
    pros::Task screen_task([this]() {
        while (true) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %.2f", chassis.getPose().x);
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %.2f", chassis.getPose().y);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %.2f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
}

void Drivetrain::drive() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    // Get joystick values
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    
    // Apply exponential curves
    double curvedThrottle = throttleCurve.curve(throttle);
    double curvedTurn = steerCurve.curve(turn);
    
    // Arcade drive
    chassis.arcade(curvedThrottle, curvedTurn);
}

void Drivetrain::run() {
    // Main run method to be called in the robot loop
    drive();
}

lemlib::Chassis& Drivetrain::get_chassis() {
    return chassis;
}

pros::MotorGroup& Drivetrain::get_left_motors() {
    return leftMotorGroup;
}

pros::MotorGroup& Drivetrain::get_right_motors() {
    return rightMotorGroup;
}
