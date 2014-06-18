#include "TutorialPlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "WhiteBoard.h"
#include "../Fundamental/RemoteTune.h"

using namespace Tribots;
using namespace std;


TutorialPlayer::TutorialPlayer (const ConfigReader& cfg) throw () {
  WBOARD->readConfigs (cfg);
  darfkicken = false;
}

DriveVector TutorialPlayer::process_drive_vector (Time t) throw () {
  DriveVector dv;

  int gamestate = 
    WorldModel::get_main_world_model().get_game_state().refstate; 
                                   // Spielzustand laut Refereebox

  if (gamestate == stopRobot) {
    darfkicken = true;
    dv.kick = 0;                   // nicht kicken
    dv.vtrans = Vec(0.,0.);        // keine translatorische Bewegung
    dv.vrot = 0;                   // keine Drehbewegung
  }else       
  if(darfkicken){
    double kicllen = 10.0;
    TUNABLE("Kick_length",&kicllen);			  
    dv.kick = 1;
    dv.klength = (int)kicllen;
    darfkicken = false;
  }
  return dv;
}


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("TutorialPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new TutorialPlayer (reader);
    }
  };
  Builder the_builder;
}
