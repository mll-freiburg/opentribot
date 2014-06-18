
#include "SharedWorldModel.h"
#include "../WorldModelFactory.h"
#include "../../Structures/Journal.h"
#include "../WorldModel.h"

using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("SharedWorldModel"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType*) throw (TribotsException,bad_alloc) {
      return new SharedWorldModel (reader);
    }
  };
  Builder the_builder;
}



Tribots::SharedWorldModel::SharedWorldModel (const ConfigReader& vr) :WorldModelTypeBase (vr),  field_geometry (vr), null_stream ("/dev/null")  {
  cout << "Construction of SharedWorldModel"<<endl;
  game_state.in_game = false;
  game_state.refstate = stopRobot;
  bool b;
  vector<double> darray (6);

    ball_loc.pos = Vec (darray[0], darray[1]);
    ball_loc.velocity = Vec (darray[2], darray[3]);
//    ball_loc.quality=0.8;
    ball_loc.pos_known=BallLocation::known;
    ball_loc.velocity_known=true;
    robot_loc.pos = Vec (darray[0], darray[1]);
    robot_loc.heading.set_deg (darray[2]);
    robot_loc.vtrans = Vec (darray[3], darray[4]);
    robot_loc.vrot = darray[5];
}

void Tribots::SharedWorldModel::reset (const Vec p) throw () {
  robot_loc.pos=p;
}

void Tribots::SharedWorldModel::reset (const Vec p, const Angle h) throw () {
  robot_loc.pos=p;
  robot_loc.heading=h;
}

SharedWorldModel::~SharedWorldModel () throw () {;}

void SharedWorldModel::startstop (bool b) throw () {
  game_state.in_game=b;
}

void SharedWorldModel::update_refbox (RefboxSignal sig) throw () {
  if (sig==Tribots::SIGstop)
    game_state.refstate = Tribots::stopRobot;
  else if (sig==Tribots::SIGstart)
    game_state.refstate = Tribots::freePlay;
}

void SharedWorldModel::set_visual_information (const VisibleObject& v, Time, unsigned int) throw () {
  if (v.object_type==VisibleObject::ball) { 
    ball_relative.pos = v.pos;
    ball_relative.valid = true;
  }
}

void SharedWorldModel::init_cycle (Time, Time) throw () {
  ball_relative.valid=false;
}

RobotLocation SharedWorldModel::get_robot (Time t) const throw () {
   
  return robot_loc;
}

BallLocation SharedWorldModel::get_ball (Time t) const throw () {
  return ball_loc;
}

ObstacleLocation SharedWorldModel::get_obstacles (Time) const throw () {
  return obstacle_loc;
}


Time SharedWorldModel::get_timestamp_latest_update () const throw () {
  return Time();
}

