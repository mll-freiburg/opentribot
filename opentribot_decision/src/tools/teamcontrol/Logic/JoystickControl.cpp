
#include "JoystickControl.h"
#include "../States/RemoteBlackboard.h"

#include <cmath>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

JoystickControl::JoystickControl () : device_name (""), the_joystick (NULL) {
  REMBB.joystick_state.joystick_okay=false;
  update ();
}

JoystickControl::~JoystickControl () {
  if (the_joystick)
    delete the_joystick;
  REMBB.joystick_state.joystick_okay=false;
}

void JoystickControl::update () {
  if (device_name!=REMBB.joystick_state.device_name) {
    device_name = REMBB.joystick_state.device_name;
    if (the_joystick) {
      delete the_joystick;
      the_joystick=NULL;
    }
    if (device_name=="Maus") {
      REMBB.joystick_state.joystick_okay=true;
    } else {
      try{
        the_joystick = new Joystick (device_name.c_str());
        REMBB.joystick_state.joystick_okay=true;
      }catch(invalid_argument&){
        the_joystick=NULL;
        REMBB.joystick_state.joystick_okay=false;
      }catch(bad_alloc&){
        the_joystick=NULL;
        REMBB.joystick_state.joystick_okay=false;
      }
    }
  }

  // Fahrtvektor berechnen:
  if (the_joystick) {
      std::vector<double> axis;
      std::vector<bool> buttons;
      axis = the_joystick->get_axis_state();
      buttons = the_joystick->get_button_state();
   
      double act_x = (REMBB.joystick_state.x_axis<axis.size() ? axis[REMBB.joystick_state.x_axis] : 0.0);
      double act_y = (REMBB.joystick_state.y_axis<axis.size() ? axis[REMBB.joystick_state.y_axis] : 0.0);
      double act_phi = (REMBB.joystick_state.phi_axis<axis.size() ? axis[REMBB.joystick_state.phi_axis] : 0.0);
 
      // ungenaue Nullstellung beruecksichtigen
      if (std::abs(act_x)<REMBB.joystick_state.null_x) act_x=0.0;
      else if (act_x>0.0) act_x = (act_x-REMBB.joystick_state.null_x)/(1.0-REMBB.joystick_state.null_x);
      else act_x = (act_x+REMBB.joystick_state.null_x)/(1.0-REMBB.joystick_state.null_x);

      if (std::abs(act_y)<REMBB.joystick_state.null_y) act_y=0.0;
      else if (act_y>0.0) act_y = (act_y-REMBB.joystick_state.null_y)/(1.0-REMBB.joystick_state.null_y);
      else act_y = (act_y+REMBB.joystick_state.null_y)/(1.0-REMBB.joystick_state.null_y);

      if (std::abs(act_phi)<REMBB.joystick_state.null_phi) act_phi=0.0;
      else if (act_phi>0.0) act_phi = (act_phi-REMBB.joystick_state.null_phi)/(1.0-REMBB.joystick_state.null_phi);
      else act_phi = (act_phi+REMBB.joystick_state.null_phi)/(1.0-REMBB.joystick_state.null_phi);

      // Koppelung von x und y beruecksichtigen
      double act_x2 = act_x*act_x;
      double act_y2 = act_y*act_y;
      double len = act_x2+act_y2;
      if (act_x2>=act_y2) len = std::sqrt (len/act_x2);
      else len = std::sqrt (len/act_y2);
      if (act_x2+act_y2>0) {
        act_x/=len;
        act_y/=len;
      }

      REMBB.joystick_state.vx = REMBB.joystick_state.max_vx*REMBB.joystick_state.sign_x*act_x;
      REMBB.joystick_state.vy = REMBB.joystick_state.max_vy*REMBB.joystick_state.sign_y*act_y;
      REMBB.joystick_state.vphi = REMBB.joystick_state.max_vphi*REMBB.joystick_state.sign_phi*act_phi;
      REMBB.joystick_state.kick = (REMBB.joystick_state.kick_button<buttons.size() ? buttons[REMBB.joystick_state.kick_button] : false);
      
      REMBB.joystick_state.activation_request = (REMBB.joystick_state.activate_button<buttons.size() ? buttons[REMBB.joystick_state.activate_button] : false);
      REMBB.joystick_state.deactivation_request = (REMBB.joystick_state.deactivate_button<buttons.size() ? buttons[REMBB.joystick_state.deactivate_button] : false);
      
  }
}
