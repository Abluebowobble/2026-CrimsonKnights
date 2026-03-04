#pragma once

#include "pros/misc.h"

namespace PORT_VALUES {
// drivetrain
constexpr int LEFT_1 = 11;
constexpr int LEFT_2 = 12;
constexpr int LEFT_3 = 13;
constexpr int RIGHT_1 = -18;
constexpr int RIGHT_2 = -19;
constexpr int RIGHT_3 = -20;

// odometry
constexpr int VERTICAL_ROTATION_SENSOR = 16;
constexpr int IMU_1 = 15; // newest
constexpr int IMU_2 = 0; // Second IMU port, old

// conveyor belt
constexpr int ENDEFFECTOR_MOTOR_PORT = 1;
constexpr int INTAKE_MOTOR_PORT = 10;

// pneumatics
constexpr int LITTLE_WILL_PNEUMATIC = 'A';
constexpr int WING_PNEUMATIC = 'B';

} // namespace PORT_VALUES

namespace CHASIS_VALUES {
constexpr double TRACKWIDTH = 11.5;
constexpr double HORIZONTAL_DRIFT = 10.5;
constexpr int RPM = 450;
constexpr double VERTICAL_TRACKING_WHEEL_OFFSET = -0.5;
} // namespace CHASIS_VALUES

namespace OPERATOR_CONSTANTS {
namespace THROTTLE {
constexpr int DEADBAND = 3;
constexpr int MIN = 10;
constexpr int MAX = 127;
constexpr double CURVE = 1;
} // namespace THROTTLE

namespace STEER {
constexpr int DEADBAND = 3;
constexpr int MIN = 20;
constexpr double CURVE = 1.000;
} // namespace STEER

constexpr double DESATURATE_BIAS = 0.45;
} // namespace OPERATOR_CONSTANTS

namespace ROBOT_PROPERTIES {
constexpr double WEIGHT_LBS = 11.0;             // robot weight in pounds
constexpr double COG_OFFSET_IN = -1.0;          // center of gravity offset from center (negative = behind center)
constexpr double DRIFT_DEG_PER_IN = 0.5;        // measured rightward heading drift per inch of travel
} // namespace ROBOT_PROPERTIES

namespace DRIVETRAIN_CONSTANTS {
namespace SNAP {
constexpr int TIMEOUT = 2000; // ms allowed for a snap turn
} // namespace SNAP
namespace LATERAL {
constexpr double KP = 10.0;
constexpr double KI = 0.0;
constexpr double KD = 3.0;
constexpr double ANTI_WINDUP_RANGE = 3.0;
constexpr double SMALL_ERROR = 1.0;
constexpr int SMALL_TIMEOUT = 100;
constexpr double LARGE_ERROR = 3.0;
constexpr int LARGE_TIMEOUT = 500;
constexpr double SLEW = 0;
} // namespace LATERAL

namespace ANGULAR {
constexpr double KP = 2.0;
constexpr double KI = 0.0;
constexpr double KD = 10.0;
constexpr double ANTI_WINDUP_RANGE = 3.0;
constexpr double SMALL_ERROR = 1.0;
constexpr int SMALL_TIMEOUT = 100;
constexpr double LARGE_ERROR = 3.0;
constexpr int LARGE_TIMEOUT = 500;
constexpr double SLEW = 0;
} // namespace ANGULAR

namespace DRIFT {
// Derived from ROBOT_PROPERTIES::DRIFT_DEG_PER_IN — use to apply
// a heading pre-correction before long straight autonomous movements
constexpr double DEG_PER_IN = 0.5;       // rightward drift magnitude (degrees per inch)
constexpr int DIRECTION = -1;            // -1 = compensate left to counter rightward drift
} // namespace DRIFT
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
constexpr int SCORE_HIGH_SPEED = 127;
constexpr int SCORE_MID_SPEED = 90;
} // namespace ENDEFFECTOR

namespace LITTLE_WILL {
constexpr auto TOGGLE = pros::E_CONTROLLER_DIGITAL_A; // moved from Y; Y is now snap-to-cardinal
} // namespace LITTLE_WILL

namespace INTAKE {
constexpr auto INTAKE = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto OUTTAKE = pros::E_CONTROLLER_DIGITAL_R2;
} // namespace INTAKE

namespace WING {
constexpr auto TOGGLE = pros::E_CONTROLLER_DIGITAL_X; // moved from A; A is now little_will
}

namespace DRIVETRAIN {
constexpr auto SNAP_CARDINAL  = pros::E_CONTROLLER_DIGITAL_Y; // snap to nearest N/E/S/W (0/90/180/270)
} // namespace DRIVETRAIN
} // namespace CONTROLLER_BUTTONS