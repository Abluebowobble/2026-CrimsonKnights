#pragma once

#include "pros/misc.h"

namespace PORT_VALUES {
// drivetrain
constexpr int LEFT_1 = 17;
constexpr int LEFT_2 = 18;
constexpr int LEFT_3 = 19;
constexpr int RIGHT_1 = -12;
constexpr int RIGHT_2 = -13;
constexpr int RIGHT_3 = -14;

// odometry
constexpr int HORIZONTAL_ROTATION_SENSOR = 16;
constexpr int VERTICAL_ROTATION_SENSOR = 6;
constexpr int IMU_1 = 0; // newest
constexpr int IMU_2 = 0; // Second IMU port, old

// conveyor belt
constexpr int ENDEFFECTOR_MOTOR_PORT = 11;
constexpr int INTAKE_MOTOR_PORT = 20;

// pneumatics
constexpr int LIL_WILL_PNEUMATIC = 1;
constexpr int WING_PNEUMATIC = 0;

} // namespace PORT_VALUES

namespace CHASIS_VALUES {
constexpr double TRACKWIDTH = 11.5;
constexpr double HORIZONTAL_DRIFT = 10.5;
constexpr int RPM = 450;
constexpr double LATERALTRACKING_WHEEL_OFFSET = 0;
constexpr double HORIZONTALTRACKING_WHEEL_OFFSET = 0;
} // namespace CHASIS_VALUES

namespace OPERATOR_CONSTANTS {
namespace THROTTLE {
constexpr int DEADBAND = 3;
constexpr int MIN = 10;
constexpr double CURVE = 1;
} // namespace THROTTLE

namespace STEER {
constexpr int DEADBAND = 3;
constexpr int MIN = 20;
constexpr double CURVE = -1.000;
} // namespace STEER

constexpr double DESATURATE_BIAS = 0.45;
} // namespace OPERATOR_CONSTANTS

namespace DRIVETRAIN_CONSTANTS {
namespace LATERAL {
constexpr double KP = 10.0;
constexpr double KI = 0.0;
constexpr double KD = 3.0;
constexpr double ANTI_WINDUP_RANGE = 3.0;
constexpr double SMALL_ERROR = 1;
constexpr int SMALL_TIMEOUT = 100;
constexpr double LARGE_ERROR = 3;
constexpr int LARGE_TIMEOUT = 500;
constexpr double SLEW = 0.0;
} // namespace LATERAL

namespace ANGULAR {
constexpr double KP = 2.0;
constexpr double KI = 0.0;
constexpr double KD = 10.0;
constexpr double ANTI_WINDUP_RANGE = 3.0;
constexpr double SMALL_ERROR = 1;
constexpr int SMALL_TIMEOUT = 100;
constexpr double LARGE_ERROR = 3;
constexpr int LARGE_TIMEOUT = 500;
constexpr double SLEW = 0;
} // namespace ANGULAR
} // namespace DRIVETRAIN_CONSTANTS

namespace VISION {
namespace RED {
constexpr double UPPER_BOUND = 20;
constexpr double LOWER_BOUND = 350;
} // namespace RED

namespace BLUE {
constexpr double UPPER_BOUND = 230; //around 224
constexpr double LOWER_BOUND = 210;

// 90-260
//
} // namespace BLUE
} // namespace VISION

namespace CONTROLLER_BUTTONS {

namespace ENDEFFECTOR {
constexpr auto SCORE_HIGH = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto SCORE_MID = pros::E_CONTROLLER_DIGITAL_L2;
} // namespace ENDEFFECTOR

namespace LIL_WILL {
constexpr auto TOGGLE = pros::E_CONTROLLER_DIGITAL_Y;
} // namespace LIL_WILL

namespace INTAKE {
constexpr auto INTAKE = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto OUTTAKE = pros::E_CONTROLLER_DIGITAL_R2;
} // namespace INTAKE

namespace WING {
constexpr auto TOGGLE = pros::E_CONTROLLER_DIGITAL_A;
}
} // namespace CONTROLLER_BUTTONS