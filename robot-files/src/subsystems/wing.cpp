#include "subsystems/wing.hpp"
#include "constants.hpp"
#include "globals.hpp"

Wing::Wing()
    : wingPneumatic(PORT_VALUES::WING_PNEUMATIC) {
    retract(); // Start retracted
}

void Wing::control(pros::Controller& master) {
    static bool lastButtonState = false;
    bool currentButtonState = master.get_digital_new_press(CONTROLLER_BUTTONS::WING::TOGGLE);
}

void Wing::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}