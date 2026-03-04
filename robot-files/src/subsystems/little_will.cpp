#include "subsystems/little_will.hpp"
#include "constants.hpp"
#include "globals.hpp"

LittleWill::LittleWill()
    : littleWillPneumatic(PORT_VALUES::LITTLE_WILL_PNEUMATIC, true),
      isExtended(false) {

}

void LittleWill::extend() {
    littleWillPneumatic.extend();
    isExtended = true;
}

void LittleWill::retract() {
    littleWillPneumatic.retract();
    isExtended = false;
}

void LittleWill::toggle() {
    littleWillPneumatic.toggle();
    isExtended = !isExtended;
}

void LittleWill::control(pros::Controller& master) {

    bool currentButtonState = master.get_digital_new_press(CONTROLLER_BUTTONS::LITTLE_WILL::TOGGLE);
    
    // Toggle on button press (rising edge detection)
    if (currentButtonState) {
        littleWillPneumatic.toggle();
        isExtended = !isExtended;
    }
}

void LittleWill::run() {
    // Use the shared global controller for operator control
    control(globals::controller);
    
}