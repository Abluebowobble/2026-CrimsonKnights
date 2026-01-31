#include "subsystems/lil_will.hpp"
#include "constants.hpp"
#include "globals.hpp"

LilWill::LilWill()
    : lilWillPneumatic(PORT_VALUES::LIL_WILL_PNEUMATIC),
      isExtended(false) {
}

void LilWill::control(pros::Controller& master) {

    bool currentButtonState = master.get_digital_new_press(CONTROLLER_BUTTONS::LIL_WILL::TOGGLE);
    
    // Toggle on button press (rising edge detection)
    if (currentButtonState) {
        lilWillPneumatic.toggle();
    }
}

void LilWill::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
    
}
