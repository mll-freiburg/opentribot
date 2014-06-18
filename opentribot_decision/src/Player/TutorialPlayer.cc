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

  int ingame=WorldModel::get_main_world_model().get_game_state().in_game;

  Vec ball=Vec(-1,-1);
	ball=WorldModel::get_main_world_model().get_ball_location(t).pos.toVec();
Frame2d abs2rel=WBOARD->getAbs2RelFrame(t);


Vec ballrelative=abs2rel*ball;


                                   // Spielzustand laut Refereebox
cout <<"In game:" <<ingame<<endl;
  if (!ingame) {
    darfkicken = true;
    dv.kick = 0;                   // nicht kicken
    dv.vtrans = Vec(1.,0.);        // keine translatorische Bewegung
    dv.vrot = 0;                   // keine Drehbewegung
  }else       
 { if(darfkicken){
    double kicllen = 10.0;
    TUNABLE("Kick_length",&kicllen);			  
    dv.kick = 1;
    dv.klength = (int)kicllen;
    darfkicken = false;
  }
dv.vtrans=ballrelative.normalize();
//dv.vtrans=Vec(1,0);
//ball.normalize();
}
cout<<"tutorialplayer"<<dv.vtrans<<endl;

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
