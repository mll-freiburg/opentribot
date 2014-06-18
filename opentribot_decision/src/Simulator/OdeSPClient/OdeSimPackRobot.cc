
#include "OdeSimPackRobot.h"
#include "../../Robot/RobotFactory.h"
#include "../../Structures/RobotProperties.h"
#include "../../Structures/Journal.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public RobotBuilder {
    static Builder the_builder;
  public:
    Builder () {
      RobotFactory::get_robot_factory ()->sign_up (string("OdeSimPackRobot"), this);
    }
    RobotType* get_robot (const std::string&, const ConfigReader& reader, RobotType*) throw (TribotsException,bad_alloc) {
      return new OdeSimPackRobot (reader);
    }
  };
  Builder the_builder;
}





Tribots::OdeSimPackRobot::OdeSimPackRobot (const ConfigReader & cfg) throw (TribotsException, bad_alloc)
{
  
  // Default-Werte setzen
  robot_properties.max_velocity                  = 1.5;
  robot_properties.max_acceleration              = 0.5;
  robot_properties.max_deceleration              = 0.8;
  robot_properties.max_rotational_velocity       = 3.0;
  robot_properties.max_rotational_acceleration   = 0.5;
  robot_properties.max_rotational_deceleration   = 1.0;
  robot_properties.max_robot_radius              = 300;
  robot_properties.min_robot_radius              = 200;
  robot_properties.kicker_width                  = 290;
  robot_properties.kicker_distance               = 190;
  robot_properties.robot_width                   = 450;
  robot_properties.robot_length                  = 450;
  robot_properties.omnidirectional_drive         = true;
  robot_properties.drive_vector_delay            = 100;
  
  // ggf. aus dem config-File Werte ueberschreiben
  robot_properties.read (cfg);

  client = 0;
  client = OdeSimPackClient::getTheClient(&cfg);

  client->sendDriveVector(DriveVector());

}

Tribots::OdeSimPackRobot::~OdeSimPackRobot() throw()
{
  ;
}

void Tribots::OdeSimPackRobot::set_drive_vector (DriveVector dv) throw (BadHardwareException, HardwareException)
{
  client->sendDriveVector(dv); 
  
  Time t_dv;
  t_dv.add_msec (robot_properties.drive_vector_delay);
  WorldModel::get_main_world_model().set_drive_vector (dv, t_dv);
  
}

void Tribots::OdeSimPackRobot::get_odometry() throw (BadHardwareException, HardwareException)
{
  ; // passiert im Weltmodell
}

Tribots::RobotProperties Tribots::OdeSimPackRobot::get_robot_properties () const throw ()
{
  return robot_properties;
}
