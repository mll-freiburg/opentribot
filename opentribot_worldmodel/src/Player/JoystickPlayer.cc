
#include <cmath>
#include "JoystickPlayer.h"
#include "PlayerFactory.h"
#include "../Structures/Journal.h"
#include "../WorldModel/WorldModel.h"

using namespace Tribots;
using namespace std;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("JoystickPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new JoystickPlayer (reader);
    }
  };
  Builder the_builder;
}





namespace {
  inline double square (const double& x) {
    return x*x;
  }
}

JoystickPlayer::JoystickPlayer (const ConfigReader& vread) throw (InvalidConfigurationException, std::bad_alloc) : js(NULL) {
  // default-Werte fuer Joystick-Achsen
  x_axis = 0;
  y_axis = 1;
  phi_axis = 3;
  x_diraxis = +1;
  y_diraxis = -1;
  phi_diraxis = -1;
  kick_button = 2;
  accel_button = 9;
  decel_button = 8;

  string confline;
  if (vread.get("Joystick::joystick_device", confline)<=0) {
    JERROR("no config line \"joystick_device\" found");
    throw Tribots::InvalidConfigurationException ("joystick_device");
  }
  try{
    js = new Joystick (confline.c_str());
  }catch(std::invalid_argument&) {
    JERROR("invalid device name in config line \"joystick_device\"");
    //    throw Tribots::InvalidConfigurationException ("joystick_device");
    js=NULL;
  }
  max_velocity = 0.5*WorldModel::get_main_world_model().get_robot_properties().max_velocity;
  max_rot_velocity = 0.5*WorldModel::get_main_world_model().get_robot_properties().max_rotational_velocity;
  previously_changed=true;

  vector<int> ii (3);
  if (vread.get ("Joystick::axis", ii)>=3) {
    x_axis = ii[0];
    y_axis = ii[1];
    phi_axis = ii[2];
  }
  if (vread.get ("Joystick::diraxis", ii)>=3) {
    x_diraxis = ii[0];
    y_diraxis = ii[1];
    phi_diraxis = ii[2];
  }
  if (vread.get ("Joystick::buttons", ii)>=3) {
    kick_button = ii[0];
    accel_button = ii[2];
    decel_button = ii[1];
  }
}

JoystickPlayer::~JoystickPlayer () throw () {
  if (js)
    delete js;
}

DriveVector JoystickPlayer::process_drive_vector (Time t) throw () {
  DriveVector dv (Vec::zero_vector, 0, false);
  if (js) {
    const std::vector<double>& axis (js->get_axis_state());
    const std::vector<bool>& buttons (js->get_button_state());
    bool currently_changed=false;

    // Annahme (wird nicht geprueft: axis.size()>=3, buttons.size()>=10)
    if (buttons[decel_button]) {   // Geschwindigkeit verringern
      currently_changed=true;
      if (!previously_changed) {
	max_velocity-=0.1*WorldModel::get_main_world_model().get_robot_properties().max_velocity;
	max_rot_velocity-=0.1*WorldModel::get_main_world_model().get_robot_properties().max_rotational_velocity;
	if (max_velocity<=0) {
	  max_velocity = 0.1*WorldModel::get_main_world_model().get_robot_properties().max_velocity;
	  max_rot_velocity = 0.1*WorldModel::get_main_world_model().get_robot_properties().max_rotational_velocity;
	}
      }
    }
    if (buttons[accel_button]) {   // Geschwindigkeit erhoehen
      currently_changed=true;
      if (!previously_changed) {
	max_velocity+=0.1*WorldModel::get_main_world_model().get_robot_properties().max_velocity;
	max_rot_velocity+=0.1*WorldModel::get_main_world_model().get_robot_properties().max_rotational_velocity;
	if (max_velocity>WorldModel::get_main_world_model().get_robot_properties().max_velocity) {
	  max_velocity = WorldModel::get_main_world_model().get_robot_properties().max_velocity;
	  max_rot_velocity = WorldModel::get_main_world_model().get_robot_properties().max_rotational_velocity;
	}
      }
    }
    previously_changed = currently_changed;  // verhindere mit diesem Mechanismus zu schnelle Aenderungen der Geschwindigkeit
    
    // Fahrtrichtung
    double dir = atan2 (y_diraxis*axis[y_axis], x_diraxis*axis[x_axis]);
    double ref_dir = floor(2*dir/M_PI+0.5)*0.5*M_PI;
    double max_len = 1.0/(cos(ref_dir-dir));  // maximal moegliche Auslenkung in Lenkrichtung
    
    double len1 = std::sqrt(axis[x_axis]*axis[x_axis]+axis[y_axis]*axis[y_axis])/max_len;  // Ansteuerung normiert auf [-1,1]
    double velf1 = (len1>=0.2 ? square((len1-0.2)/0.8)/len1 : 0.0);  // ungenaue Nullstellung und nichtlineare Ansteuerung eingerechnet
    dv.vtrans = velf1*max_velocity*Vec::unit_vector_x.rotate (Angle::rad_angle (dir));
    
    double len2 = std::abs(axis[phi_axis]);
    double velf2 = (len2>=0.2 ? square((len2-0.2)/0.8)/len2 : 0.0);
    dv.vrot = phi_diraxis*velf2*axis[phi_axis]*max_rot_velocity;
    
    // Kicker
    dv.kick = buttons[kick_button];
    
  }
  

  return dv;
}

