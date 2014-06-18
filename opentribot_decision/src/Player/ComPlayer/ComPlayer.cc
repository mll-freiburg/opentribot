
#include "../../Fundamental/Time.h"
#include "../../Structures/GameState.h"
#include "../../Structures/RobotData.h"
#include "../../Structures/Journal.h"
#include "../../WorldModel/WorldModel.h"
#include "ComPlayer.h"
#include "../PlayerFactory.h"
#include <cmath>

#include <iostream>
#include <sstream>

using namespace std;
using namespace Tribots;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("ComPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new ComPlayer (reader);
    }
  };
  Builder the_builder;
}



Tribots::ComPlayer::ComPlayer (const ConfigReader& vread) throw ()  
{
  //if (vread.get ("RLPlayer::home_y_pos", u)>0 && u>0)
  //  home_y=u;

  serv.init(7010);

  if (!serv.wait_for_client())
    {
      JERROR("Failed to init a ComServer ... exiting!\n");
      exit(0);
    }
  
}

Tribots::ComPlayer::~ComPlayer () throw () {;}


DriveVector Tribots::ComPlayer::process_drive_vector (Time tt) throw () 
{
  DriveVector ret;
  const GameState& game_state (WorldModel::get_main_world_model().get_game_state ());
 
  if (game_state.refstate==stopRobot) {
    return ret;
  }

  Time t;
  
  // Informationen aus dem Weltmodell beschaffen
  //const FieldGeometry& field_geometry (WorldModel::get_main_world_model().get_field_geometry());
  RobotLocation robot_location (WorldModel::get_main_world_model().get_robot_location (tt));
  BallLocation  ball_location (WorldModel::get_main_world_model().get_ball_location (tt));
  RobotData     robot_data (WorldModel::get_main_world_model().get_robot_data (t));
  // int game_state (WorldModel::get_main_world_model().get_game_state ());

  Time comTime;

  serv.putTime(t);
  serv.putRobotLocation(robot_location);
  serv.putBallLocation(ball_location);
  serv.putRobotData(robot_data);
  serv.putGameState(game_state.refstate);
  serv.send();

  Time t_com;
  serv.receive_all(500);
  serv.getTime(t_com);
  if (game_state.refstate!=stopRobot)
    serv.getDriveVector(ret);
  
  std::cout << "Time for ComPlayer send and receive (+computation of new dv in client) : " << comTime.elapsed_usec() << " usec\n\r";
  return ret;
}


