#include "SaschasPassPlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "WhiteBoard.h"
#include "../Fundamental/RemoteTune.h"
#include <cmath>

using namespace Tribots;
using namespace std;

SaschasPassPlayer::SaschasPassPlayer (const ConfigReader& cfg) throw () {
  WBOARD->readConfigs (cfg);
  lastKick.update();
}

DriveVector SaschasPassPlayer::process_drive_vector (Time t) throw () {
  DriveVector dv;

  int gamestate = 
    WorldModel::get_main_world_model().get_game_state().refstate; 
                                   // Spielzustand laut Refereebox
  const RobotLocation& robot = MWM.get_robot_location(t);
  const BallLocation& ball = MWM.get_ball_location(t);
  
  Vec relBall = WBOARD->getAbs2RelFrame(t) * ball.pos.toVec();
  bool inKickRange = 
    relBall.y > 0 && relBall.y < 500. && 
    fabs(relBall.x) < 100.;
  LOUT << "Relative Ballposition: " << relBall << "\r\n";

  if (gamestate == stopRobot) {
    dv.kick = 0;                   // nicht kicken
    dv.vtrans = Vec(0.,0.);        // keine translatorische Bewegung
    dv.vrot = 0;                   // keine Drehbewegung
  }
  else {      
    if (lastKick.elapsed_sec() > 1 && inKickRange) {			  
      dv.kick = 1;
      dv.klength = 100;
      lastKick.update();
    }
  }
  return dv;
}


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("SaschasPassPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new SaschasPassPlayer (reader);
    }
  };
  Builder the_builder;
}
