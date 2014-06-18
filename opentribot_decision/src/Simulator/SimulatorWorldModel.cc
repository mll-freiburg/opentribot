
#include "SimulatorWorldModel.h"
#include "../WorldModel/WorldModelFactory.h"
#include "../Structures/Journal.h"
#include "../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("Simulator"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType*) throw (TribotsException,bad_alloc) {
      return new SimulatorWorldModel (reader);
    }
  };
  Builder the_builder;
}




SimulatorWorldModel::SimulatorWorldModel (const ConfigReader& reader) throw (std::bad_alloc, Tribots::InvalidConfigurationException) : WorldModelTypeBase (reader), the_sim_client (NULL), ball_filter (reader), obstacle_filter (reader, get_field_geometry()), ball_vision_radius (1000000), obstacle_vision_radius (1000000) {
  string confline;
  if (reader.get("Simulator::robot_config_file", confline)<=0) {
    JERROR("no config line \"Simulator::robot_config_file\" found");
    throw Tribots::InvalidConfigurationException("Simulator::robot_config_file");
  }
  try{
    the_sim_client=SimClient::get_sim_client (confline.c_str());
  }catch(std::invalid_argument&){
    JERROR("cannot connect to simulator");
    throw Tribots::InvalidConfigurationException("Simulator::robot_config_file");
  }
  reader.get("SimulatorVision::ball_vision_radius", ball_vision_radius);
  reader.get("SimulatorVision::obstacle_vision_radius", obstacle_vision_radius);
}

SimulatorWorldModel::~SimulatorWorldModel () throw () {;}

RobotLocation SimulatorWorldModel::get_robot (Time t) const throw () {
  RobotLocation dest = odobox.add_movement (cpos, cpos_time, t);
  dest.stuck = stuck_sensor.get_stuck_location(dest.pos, dest.vtrans);
  return dest;
}

BallLocation SimulatorWorldModel::get_ball (Time t) const throw () {
  return ball_filter.get(t);
}

ObstacleLocation SimulatorWorldModel::get_obstacles (Time) const throw () {
  return obstacle_filter.get_with_poles_and_stuck();
}

unsigned int SimulatorWorldModel::update_localisation () throw () {
  the_sim_client->update();
  double own = -get_own_half();
  Angle add = (own<0.0 ? Angle::half : Angle::zero);
  cpos.pos = own*the_sim_client->robot_position;
  cpos.heading = the_sim_client->robot_heading+add;
  cpos_time = the_sim_client->timestamp;

  // Ball eintragen:
  if ((the_sim_client->ball_position-the_sim_client->robot_position).length()<ball_vision_radius) {
    add_ball_relative ((the_sim_client->ball_position-the_sim_client->robot_position).rotate(-the_sim_client->robot_heading));
  }

  // Hindernisse eintragen:
  std::vector<Vec>::iterator oit = the_sim_client->obstacle_positions.begin();
  std::vector<Vec>::iterator oit_end = the_sim_client->obstacle_positions.end();
  while (oit<oit_end) {
    if (((*oit)-the_sim_client->robot_position).length()<obstacle_vision_radius) {
      Vec opos = ((*oit)-the_sim_client->robot_position).rotate(-the_sim_client->robot_heading);
      double len = opos.length();
      if (len>200)
        opos*=(1.0-150.0/len);  // Hindernis etwas zum Betrachter herruecken
      add_obstacle_relative (opos, 300);  // Hindernisbreite pauschal mit 300mm angenommen
    }
    oit++;
  }
  
  if (visbox.size()>0) {
    VisibleObjectList visboxball = visbox[0].get_balls();
    visboxball.timestamp = cpos_time;
    ball_filter.update (visboxball, get_robot (cpos_time));
    
    VisibleObjectList visboxobs = visbox[0].get_obstacles();
    visboxobs.timestamp = cpos_time;
    obstacle_filter.update (visboxobs, get_robot(cpos_time), get_ball(cpos_time));
  }

  // kommunizierter Ball
  const string cb = get_message_board().scan_for_prefix ("Ball: ");
  if (cb.length()>0) {
    vector<string> parts;
    split_string (parts, cb);
    if (parts.size()>=3) {
      Vec bp;
      string2double (bp.x, parts[1]);
      string2double (bp.y, parts[2]);
      ball_filter.comm_ball (bp);
    }
  }

  return 1|2|4;
}

void SimulatorWorldModel::reset () throw () {;}

void SimulatorWorldModel::reset (const Vec) throw () {;}

Time SimulatorWorldModel::get_timestamp_latest_update () const throw () { return cpos_time; }

void SimulatorWorldModel::init_cycle (Time t1, Time t2) throw () {
  // pro Forma einen Linienpunkt uebergeben, um die Zeitstempel der visboxen zu aktualisieren
  WorldModelTypeBase::init_cycle (t1, t2);
  Time now;
  VisibleObject vobj (Vec(0,0), VisibleObject::white_line);
  this->set_visual_information (vobj, now, 0);
}
