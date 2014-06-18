
#include "SimulatedTribot.h"
#include "../Robot/RobotFactory.h"
#include "../Structures/Journal.h"
#include "../WorldModel/WorldModel.h"
#include <cmath>

#define DEG2RAD( __X__ ) (__X__ * M_PI / 180.0)

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public RobotBuilder {
    static Builder the_builder;
  public:
    Builder () {
      RobotFactory::get_robot_factory ()->sign_up (string("Simulator"), this);
    }
    RobotType* get_robot (const std::string&, const ConfigReader& reader, RobotType*) throw (TribotsException,bad_alloc) {
      return new SimulatedTribot (reader);
    }
  };
  Builder the_builder;
}


//--- helper functions

/** Compute direct kinematic for OmniRobot Tribot **/
void direct_kin(double w1, double w2 , double w3 , double& xmp , double& ymp , double& phip)
{
  static double L1 = 0.185;
  static double L2 = L1;
  static double sindelta = sin(DEG2RAD(30.0));
  static double cosdelta = cos(DEG2RAD(30.0));
  static double wheelRadius = 0.04;

  double h1, h2, h3;

  h1 = -L2 / (2.0 * (L1 + sindelta * L2));
  h2 = -L2 / (2.0 * (L1 + sindelta * L2));
  h3 =  L1 / ( L1 + sindelta *  L2);
  xmp = wheelRadius * ( h1 * w1 + h2 *  w2 + h3 * w3);

  h1 =  1.0 / (2*cosdelta);
  h2 = -1.0 / (2*cosdelta);
  h3 =  0;
  ymp = wheelRadius * ( h1 * w1 + h2 * w2 + h3 * w3);

  h1 =  1.0 / (2.0 * (L1 + sindelta * L2));
  h2 =  1.0 / (2.0 * (L1 + sindelta * L2));
  h3 =  sindelta /   (L1 + sindelta * L2);
  phip = wheelRadius * ( h1 * w1 + h2 * w2 + h3 * w3);
}

//-------------------


SimulatedTribot::SimulatedTribot (const ConfigReader& vr) throw (std::bad_alloc, Tribots::InvalidConfigurationException) : the_sim_client (NULL), drv(15) {
  // Pointer auf den SimClient beschaffen
  string confline;
  if (vr.get("Simulator::robot_config_file", confline)<=0) {
    JERROR("no config line \"robot_config_file\" found");
    throw Tribots::InvalidConfigurationException("robot_config_file");
  }
  try{
    the_sim_client=SimClient::get_sim_client (confline.c_str());
  }catch(std::invalid_argument&){
    JERROR("cannot connect to simulator");
    throw Tribots::InvalidConfigurationException("robot_config_file");
  }

  // Robotereigenschaften setzen/lesen
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
  robot_properties.drive_vector_delay              = 0;

  // ggf. aus dem config-File Werte ueberschreiben
  robot_properties.read (vr);
}

SimulatedTribot::~SimulatedTribot () throw () {;}

RobotProperties SimulatedTribot::get_robot_properties () const throw () {
  return robot_properties;
}

void SimulatedTribot::set_drive_vector (DriveVector dv) throw () {
  DriveVector dv_tmp = dv;

  switch (dv.mode) {
  case ROBOTVELOCITY:
    // nothing to do here, just keep original dv
    dv_tmp = dv;
    break;
  case WHEELVELOCITY:
    {
      // compute relative dv velocities based on kinematic helper function
      double xmp, ymp, phip;
      direct_kin(dv.vaux[0], dv.vaux[1] , dv.vaux[2] , xmp , ymp , phip);
      dv_tmp = DriveVector(xmp , ymp , phip, dv.kick , ROBOTVELOCITY , dv.klength);
    }
    break;
  case MOTORVOLTAGE:
    // we would need a dynamic model here ... not implemented yet
  default:
    JERROR("Not handled DriveVector mode in Simulator ! ");
  };

  TDriveVector nd;
  nd.dv = dv_tmp;
  drv.get()=nd;
  drv.step();
  nd.timestamp.add_msec (robot_properties.drive_vector_delay);
  MWM.set_drive_vector (dv_tmp, nd.timestamp);

  Time now;
  int index=-1;
  unsigned int c=0;
  while (drv[index].timestamp>now && c++<drv.size())
    index--;
  if (drv[index].timestamp<=now)
    the_sim_client->set_drive_vector (drv[index].dv);

  RobotData rd;
  rd.BoardID = -1;
  sprintf(rd.robotIdString,"SIMULATION");

  // VORSICHT dies ist eine SCHNELLE Lösung und sollte durch ein allgemein vorhandenes Modell ersetzt werden das man hier abfragen kann ohne den Code fix einzucompilieren!!

  Time tt;
  RobotLocation robot_location (WorldModel::get_main_world_model().get_slfilter_robot_location (tt));

  robot_location.vtrans/=robot_location.heading;

  float sindelta=sin((M_PI/180.0) * 30);
  float cosdelta=cos((M_PI/180.0) * 30);
  float L1_m = 0.185;
  float L2_m = 0.185;
  float R_m = 0.04;
  rd.wheel_vel[0] = 1.0/R_m * (-sindelta * robot_location.vtrans.x + cosdelta * robot_location.vtrans.y  + L1_m * robot_location.vrot);
  rd.wheel_vel[1] = 1.0/R_m * (-sindelta * robot_location.vtrans.x - cosdelta * robot_location.vtrans.y  + L1_m * robot_location.vrot);
  rd.wheel_vel[2] = 1.0/R_m * (            robot_location.vtrans.x                  + L2_m * robot_location.vrot);

  rd.robot_vel[0] = robot_location.vtrans.x;
  rd.robot_vel[1] = robot_location.vtrans.y;
  rd.robot_vel[2] = robot_location.vrot;
  
  Time      rd_time;
  WorldModel::get_main_world_model().set_robot_data(rd, rd_time);
}

