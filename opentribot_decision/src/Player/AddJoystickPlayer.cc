
#include "AddJoystickPlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "../Structures/GameState.h"
#include "../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("AddJoystickPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType* pt) throw (TribotsException,bad_alloc) {
      return new AddJoystickPlayer (reader, pt);
    }
  };
  Builder the_builder;
}





AddJoystickPlayer::AddJoystickPlayer (const ConfigReader& vr, PlayerType* pl) throw (InvalidConfigurationException, std::bad_alloc) : JoystickPlayer (vr), the_elementary_player (pl), num_error (0) {
  if (!pl)
    throw InvalidConfigurationException ("player_type");
  start_button = 1;
  stop_button = 0;
  vector<unsigned int> ii (5);
  if (vr.get ("Joystick::buttons", ii)>=5) {
    start_button = ii[3];
    stop_button = ii[4];
  }
}

AddJoystickPlayer::~AddJoystickPlayer () throw () {
  delete the_elementary_player;
}

DriveVector AddJoystickPlayer::process_drive_vector (Time t) throw () {
  DriveVector dest = the_elementary_player->process_drive_vector (t);  // damit irgendwelche zyklischen Aufgaben erledigt werden koennen
  
  if (MWM.get_game_state().refstate!=stopRobot) {
    latest_time_nonstop.update();
  } else if (latest_time_nonstop.elapsed_msec()>=500) {  // Abfrage, um ein Ausrollen zu ermoeglichen
    bool global=false;
    std::string sd = MWM.get_message_board().scan_for_prefix ("JoyDrv:");
    if (sd.length()==0) {
      global=true;
      sd = MWM.get_message_board().scan_for_prefix ("JoyDrvGlob:");
    }
    std::vector<std::string> parts;
    split_string (parts, sd);
    bool error=true;
    if (parts.size()>=5) {
      error=false;
      DriveVector res;
      error |= !string2double (res.vtrans.x, parts[1]);
      error |= !string2double (res.vtrans.y, parts[2]);
      error |= !string2double (res.vrot, parts[3]);
      error |= !string2uint (res.kick, parts[4]);
      if (parts.size() >= 6) {
        int klength=100;
        error |= !string2int(klength, parts[5]);
        res.klength = (unsigned int) klength;
      }
      if (global)
        res.vtrans/=(MWM.get_robot_location (t).heading + (MWM.get_own_half()==-1 ? Angle::half : Angle::zero));
      if (!error)
        remoteCtrDrv=res;
    }
    if (error)
      num_error++;
    else
      num_error=0;
    if (num_error>=30) {
      num_error=30;
      dest = JoystickPlayer::process_drive_vector (t);
    } else
      dest = remoteCtrDrv;
  }

  return dest;
}

const char* AddJoystickPlayer::get_role () throw () { return the_elementary_player->get_role(); }
bool AddJoystickPlayer::set_role (const char* s) throw () { return the_elementary_player->set_role(s); }
const std::vector<std::string>& AddJoystickPlayer::get_list_of_roles () throw ()  { return the_elementary_player->get_list_of_roles(); }
void AddJoystickPlayer::updateTactics (const TacticsBoard& tb) throw () { the_elementary_player->updateTactics (tb); }
