#ifndef OI_H
#define OI_H

#include <Buttons/JoystickButton.h>
#include <Joystick.h>
class OI {
public:

	OI();
	frc::Joystick* GetJoystick();




private:
	frc::Joystick joy { 0 };

	// Create some buttons
	frc::JoystickButton camera_left { &joy, 4 };
	frc::JoystickButton camera_right { &joy, 5 };

};

#endif  // OI_H
