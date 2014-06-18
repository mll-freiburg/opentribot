
#include "OdeSimPackWorldModel.h"
#include "../../WorldModel/WorldModelFactory.h"
#include "../../Structures/Journal.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("OdeSimPackWorldModel"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType*) throw (TribotsException,bad_alloc) {
      return new OdeSimPackWorldModel (reader);
    }
  };
  Builder the_builder;
}




OdeSimPackWorldModel::OdeSimPackWorldModel (const ConfigReader& reader) throw (std::bad_alloc, Tribots::InvalidConfigurationException) 
  : WorldModelTypeBase (reader), client (NULL), ball_filter (reader), obstacle_filter (reader, get_field_geometry()) 
{
  try {
    client = OdeSimPackClient::getTheClient(&reader);
  } 
  catch (Tribots::TribotsException &e) {
    stringstream msg;
    msg << "Connect to Simulator failed: " << e.what();
    throw Tribots::InvalidConfigurationException(msg.str().c_str());
  }
}


OdeSimPackWorldModel::~OdeSimPackWorldModel () throw () {;}

RobotLocation OdeSimPackWorldModel::get_robot (Time t) const throw () 
{
  RobotLocation dest = odobox.add_movement (cpos, cpos_time, t);
  dest.stuck = stuck_sensor.get_stuck_location(dest.pos, dest.vtrans);
  return dest;
}

BallLocation OdeSimPackWorldModel::get_ball (Time t) const throw () {
  return ball_filter.get(t);
}

ObstacleLocation OdeSimPackWorldModel::get_obstacles (Time) const throw () {
  return obstacle_filter.get_with_poles_and_stuck();
}

unsigned int OdeSimPackWorldModel::update_localisation () throw () {
  
  double own = -get_own_half();
  Angle add = (own<0.0 ? Angle::half : Angle::zero);
  
  const OdeSimPackClientData& SimData = client->getLastSimData();

  cpos.pos      = own * Vec(SimData.RobotPos[0] * 1000.0, SimData.RobotPos[1] * 1000.0);
  cpos.heading  = Angle(SimData.RobotPos[2]) + add;
  cpos_time     = SimData.timestamp;
  

  Vec ballpos = own * Vec(SimData.BallAbsPos[0] * 1000.0, SimData.BallAbsPos[1] * 1000.0);

  ball_filter.update (ballpos , SimData.timestamp, cpos.pos);


  // Ball eintragen:
  //if ((the_sim_client->ball_position-the_sim_client->robot_position).length()<ball_vision_radius) {
    add_ball_relative ((ballpos-cpos.pos).rotate(-cpos.heading));
  //}

  odobox.add_odometry (DriveVector(SimData.RobotRelVel[0], SimData.RobotRelVel[1], SimData.RobotRelVel[2]), SimData.timestamp);

  vector<double> olwidth (0);
  vector<Vec> olpos (SimData.Obstacles.size()/2);
  olwidth.assign (SimData.Obstacles.size()/2, 600);   // Standardbreite 600mm
  for (unsigned int i=0; i<SimData.Obstacles.size()/2; i++)
    olpos[i]=own * Vec(SimData.Obstacles[i*2+0] * 1000.0, SimData.Obstacles[i*2+1] * 1000.0);
  obstacle_filter.update (olpos, olwidth, SimData.timestamp);
  return 1|2|4;
}

void OdeSimPackWorldModel::reset () throw () {;}

void OdeSimPackWorldModel::reset (const Vec) throw () {;}

Time OdeSimPackWorldModel::get_timestamp_latest_update () const throw () { return cpos_time; }

void OdeSimPackWorldModel::init_cycle (Time t1, Time t2) throw () {
  // pro Forma einen Linienpunkt uebergeben, um die Zeitstempel der visboxen zu aktualisieren
  WorldModelTypeBase::init_cycle (t1, t2);
  Time now;
  VisibleObject vobj (Vec(0,0), VisibleObject::white_line);
  this->set_visual_information (vobj, now, 0);
}

