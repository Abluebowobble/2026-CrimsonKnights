#include "subsystems/lil_will.hpp"
#include "constants.hpp"
#include "globals.hpp"

LilWill::LilWill()
    : lilWillPneumatic(PORT_VALUES::LIL_WILL_PNEUMATIC),
      isExtended(false) {
    retract(); // Start retracted
}

void LilWill::extend() {
    lilWillPneumatic.set_value(true);
    isExtended = true;
}

void LilWill::retract() {
    lilWillPneumatic.set_value(false);
    isExtended = false;
}

void LilWill::toggle() {
    if (isExtended) {
        retract();
    } else {
        extend();
    }
}

void LilWill::control(pros::Controller& master) {
    static bool lastButtonState = false;
    bool currentButtonState = master.get_digital(CONTROLLER_BUTTONS::LIL_WILL::TOGGLE);
    
    // Toggle on button press (rising edge detection)
    if (currentButtonState && !lastButtonState) {
        toggle();
    }
    
    lastButtonState = currentButtonState;
}

void LilWill::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
}
