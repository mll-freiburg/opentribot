
#ifndef TribotsTools_JoystickControl_h
#define TribotsTools_JoystickControl_h

#include <string>
#include "../../../../Fundamental/Joystick.h"

namespace TribotsTools {

  /** Klasse, um den Joystick auszulesen */
  class JoystickControl {
  public:
    JoystickControl ();
    ~JoystickControl ();
    void update ();

  private:
    std::string device_name;
    Tribots::Joystick* the_joystick;
  };

}

#endif
