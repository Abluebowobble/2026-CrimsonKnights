#include "subsystems/lil_will.hpp"
#include "constants.hpp"
#include "globals.hpp"

LilWill::LilWill()
    : lilWillPneumatic(PORT_VALUES::LIL_WILL_PNEUMATIC),
      isExtended(false) {
    retract(); // Start retracted
}

void LilWill::control(pros::Controller& master) {
    static bool lastButtonState = false;
    bool currentButtonState = master.get_digital_new_press(CONTROLLER_BUTTONS::LIL_WILL::TOGGLE);
    
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
