#include "subsystems/wing.hpp"
#include "constants.hpp"
#include "globals.hpp"

Wing::Wing()
    : wingPneumatic(PORT_VALUES::WING_PNEUMATIC),
      isExtended(false) {
    retract(); // Start retracted
}

void Wing::extend() {
    wingPneumatic.set_value(true);
    isExtended = true;
}

void Wing::retract() {
    wingPneumatic.set_value(false);
    isExtended = false;
}

void Wing::toggle() {
    if (isExtended) {
        retract();
    } else {
        extend();
    }
}

void Wing::control(pros::Controller& master) {
    static bool lastButtonState = false;
    bool currentButtonState = master.get_digital(CONTROLLER_BUTTONS::WING::TOGGLE);
    
    // Toggle on button press (rising edge detection)
    if (currentButtonState && !lastButtonState) {
        toggle();
    }
    
    lastButtonState = currentButtonState;
}

void Wing::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}