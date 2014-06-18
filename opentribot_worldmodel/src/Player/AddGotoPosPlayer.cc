
#include "AddGotoPosPlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("AddGotoPosPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType* pt) throw (TribotsException,bad_alloc) {
      return new AddGotoPosPlayer (reader, pt);
    }
  };
  Builder the_builder;
}






AddGotoPosPlayer::AddGotoPosPlayer (const ConfigReader& cfg, PlayerType* pt) throw (InvalidConfigurationException, std::bad_alloc) : the_elementary_player (pt), is_active(false), arrived(false) {
  if (pt==NULL)
    throw InvalidConfigurationException("AddGotoPosPlayer ohne richtigen Player");
  goto_pos_skill.set_dynamics (1, 2, 2, 2);
}

AddGotoPosPlayer::~AddGotoPosPlayer () throw () {
  delete the_elementary_player;
}

DriveVector AddGotoPosPlayer::process_drive_vector (Time t) throw () {
  DriveVector dest = the_elementary_player->process_drive_vector (t);
  
  string prline = MWM.get_message_board().scan_for_prefix ("GotoPos:");
  vector<string> parts;
  split_string (parts, prline);
  if (parts.size()>=4) {
    // pruefen, ob neue Position vorgegeben wurde
    double tgh;
    string2double (target_pos.x, parts[1]);
    string2double (target_pos.y, parts[2]);
    string2double (tgh, parts[3]);
    target_heading = Angle::deg_angle (tgh);
    int oh = MWM.get_own_half();
    goto_pos_skill.init (oh*target_pos, target_heading + (oh>0 ? Angle::zero : Angle::half), true);
    is_active=true;
    arrived=false;
    latest_refstate=MWM.get_game_state().refstate;
  }
  
  if (is_active) {
    // pruefen, ob GotoPos weiterhin aktiv sein soll
    RefereeState crefstate = MWM.get_game_state().refstate;
    if (latest_refstate!=crefstate) { // TODO: das erscheint mir falsch zu sein!!!
  /*    if (crefstate==postOpponentKickOff || crefstate==postOpponentGoalKick || crefstate==postOpponentCornerKick || crefstate==postOpponentThrowIn || crefstate==postOpponentFreeKick || crefstate==postOpponentPenalty)
	latest_refstate = crefstate; */
      if (crefstate==postOpponentKickOff || crefstate==postOpponentGoalKick || crefstate==postOpponentCornerKick || crefstate==postOpponentThrowIn || crefstate==postOpponentFreeKick || crefstate==postOpponentPenalty)
        latest_refstate = crefstate;
      else
	is_active=false;
    }
  }

  if (is_active) {
    if (!arrived) {
      // pruefen, ob stuck oder am Ziel angekommen
      const RobotLocation& rloc = MWM.get_robot_location (t);
      if (rloc.stuck()) arrived=true;   // zwar nicht angekommen, aber stuck
      if (((rloc.pos-MWM.get_own_half()*target_pos).length()<100) && (rloc.heading-target_heading-(MWM.get_own_half()>0 ? Angle::zero : Angle::half)).in_between (-Angle::deg_angle (5), Angle::deg_angle(5))) arrived=true; // angekommen    
    }
    if (arrived) {
      is_active = true; // TODO: neu! 
      
      dest.vtrans = Vec::zero_vector;
      dest.vrot = 0;
      dest.kick = false;
    } else {
      // wenn aktiv, dann DriveVector von GotoPosSkill holen
      dest = goto_pos_skill.getCmd (t);
      dest.kick=false;
    }
  }

  return dest;
}

const char* AddGotoPosPlayer::get_role () throw () { return the_elementary_player->get_role(); }
bool AddGotoPosPlayer::set_role (const char* s) throw () { return the_elementary_player->set_role(s); }
const std::vector<std::string>& AddGotoPosPlayer::get_list_of_roles () throw ()  { return the_elementary_player->get_list_of_roles(); }
void AddGotoPosPlayer::updateTactics (const TacticsBoard& tb) throw () { the_elementary_player->updateTactics (tb); }
