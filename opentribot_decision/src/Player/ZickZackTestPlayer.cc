#include "ZickZackTestPlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "WhiteBoard.h"
#include "../Fundamental/RemoteTune.h"

using namespace Tribots;
using namespace std;


ZickZackTestPlayer::ZickZackTestPlayer (const ConfigReader& cfg) throw () {
  WBOARD->readConfigs (cfg); state = 0;
}


DriveVector ZickZackTestPlayer::process_drive_vector (Time t) throw () {
  DriveVector dv;
  
  enum {STOP=0, START, ZICK, TOGOAL, MOVEGOAL, PAUSE};
  
  int gamestate = 
    WorldModel::get_main_world_model().get_game_state().refstate; 
                                   // Spielzustand laut Refereebox

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  const RobotLocation& robot = MWM.get_robot_location(t);
  Vec point1(0, fgeom.field_length/6);
  Vec point2(0, fgeom.field_length/2.2);
  if (gamestate == stopRobot) {
    state = STOP;
    dv.kick = 0;                   // nicht kicken
    dv.vtrans = Vec(0.,0.);        // keine translatorische Bewegung
    dv.vrot = 0;                   // keine Drehbewegung
  }
  else {
    Vec target = robot.pos;
    double vel = 0;
    switch (state) {
      case STOP: 
        target = point1;
        vel = 1.;
        if ((target-robot.pos).length() < 200.) { 
          state = START;
          timer.update();
        }
        break;
      case START:
        vel = 0.;
        if (timer.elapsed_sec() >= 4) {
          state = ZICK;
          counter = 0;
        }
        break;
      case ZICK:
        target = counter % 2 == 0 ? point2 : point1;
        vel = 1. + counter * .3;
        if ((target-robot.pos).length() < 300.) {
          counter++;
        }
        if (counter == 7) {
          state = TOGOAL;
        }
        break;
      case TOGOAL:
        target = Vec(0, fgeom.field_length/2. + 500.);
        vel = .8;
        if ((target-robot.pos).length() < 200.) {
          state = MOVEGOAL;
          timer.update();
        }
        break;
      case MOVEGOAL:
        target = Vec(0, fgeom.field_length/2. +1000.);
        vel = 1. + timer.elapsed_sec() * .5;
        if (timer.elapsed_sec() >= 6) {
          state = PAUSE;
          timer.update();
        }
        break;
      case PAUSE:
        vel = 0.;
        if (timer.elapsed_sec() >= 5) {
          state = STOP;
        }
        break;
    }
    cerr << "State: " << state << "   target: " << target << "      \n\r"; 
    dv.vtrans = ((target-robot.pos).normalize() / robot.heading) * vel;
    cerr << "Vtrans: " << dv.vtrans << "\n\r";
    dv.vrot = 0;
    dv.kick = 0;
  }
  return dv;
}


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("ZickZackTestPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new ZickZackTestPlayer (reader);
    }
  };
  Builder the_builder;
}
