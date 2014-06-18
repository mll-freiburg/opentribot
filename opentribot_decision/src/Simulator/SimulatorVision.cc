
#include "SimulatorVision.h"
#include "../ImageProcessing/VisionFactory.h"
#include "../Fundamental/random.h"
#include "../Structures/Journal.h"
#include "../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public VisionBuilder {
    static Builder the_builder;
  public:
    Builder () {
      VisionFactory::get_vision_factory ()->sign_up (string("Simulator"), this);
    }
    VisionType* get_vision (const std::string&, const std::string&, const ConfigReader& reader, VisionType*) throw (TribotsException,bad_alloc) {
      return new SimulatorVision (reader);
    }
  };
  Builder the_builder;
}



namespace {

  inline void bound (double& x, double lb, double ub) {
    x = (x<lb ? lb : (x>ub ? ub : x));
  }
  
}



SimulatorVision::SimulatorVision (const ConfigReader& reader) throw (std::bad_alloc, Tribots::InvalidConfigurationException) : the_sim_client (NULL), logtruestream (NULL) {
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
  
  noise_level = 0.001;
  line_mis_probability = 0;
  goal_mis_probability=0;
  ball_mis_probability=0;
  obstacle_mis_probability=0;
  line_vision_radius = 1e100;
  ball_vision_radius = 1e100;
  obstacle_vision_radius = 1e100;
  unsigned int num_scanlines = 30;
  double d;
  if (reader.get ("SimulatorVision::noise_level",d)>=1 && d>0)
    noise_level=d;
  if (reader.get ("SimulatorVision::line_mis_probability",d)>=1 && d>=0 && d<=1)
    line_mis_probability=d;
  if (reader.get ("SimulatorVision::goal_mis_probability",d)>=1 && d>=0 && d<=1)
    goal_mis_probability=d;
  if (reader.get ("SimulatorVision::ball_mis_probability",d)>=1 && d>=0 && d<=1)
    ball_mis_probability=d;
  if (reader.get ("SimulatorVision::obstacle_mis_probability",d)>=1 && d>=0 && d<=1)
    obstacle_mis_probability=d;
  if (reader.get ("SimulatorVision::line_vision_radius",d)>=1 && d>=0)
    line_vision_radius=d;
  if (reader.get ("SimulatorVision::ball_vision_radius",d)>=1 && d>=0)
    ball_vision_radius=d;
  if (reader.get ("SimulatorVision::obstacle_vision_radius",d)>=1 && d>=0)
    obstacle_vision_radius=d;
  reader.get ("SimulatorVision::num_scanlines",num_scanlines);
  simulate_gyro=simulate_odo=false;
  reader.get ("SimulatorVision::simulate_gyroscope", simulate_gyro);
  reader.get ("SimulatorVision::simulate_odometry", simulate_odo);
  odo_x_noise=odo_y_noise=odo_phi_noise=0;
  gyro_phi_noise=0.01*nrandom();
  
  // Scanlinien erzeugen
  double rect [] = { 390, 540, 730, 930, 1200, 1500 };
  for (unsigned int i=0; i<6; i++) {
    scanlines.push_back (Line(Vec(rect[i],rect[i]), Vec(rect[i],-rect[i])));
    scanlines.push_back (Line(Vec(-rect[i],-rect[i]), Vec(rect[i],-rect[i])));
    scanlines.push_back (Line(Vec(-rect[i],-rect[i]), Vec(-rect[i],rect[i])));
    scanlines.push_back (Line(Vec(rect[i],rect[i]), Vec(-rect[i],rect[i])));
  }
  scanlines.push_back (Line(Vec(0,0), Vec::unit_vector(Angle::zero)));
  double stepsize=M_PI;
  unsigned int n=1;
  while (n<num_scanlines) {
    double angle = stepsize/2;
    while (angle<M_PI) {
      scanlines.push_back (Line(Vec(0,0), Vec::unit_vector(Angle::rad_angle (angle))));
      angle+=stepsize;
      n++;
    }
    stepsize/=2;
  }

  // Linien eintragen
  const FieldGeometry& fg (WorldModel::get_main_world_model().get_field_geometry());
  fg.make_lineset (lines, arcs);

  bool logtrue=false;
  reader.get ("SimulatorVision::log_true_position", logtrue);
  if (logtrue)
    logtruestream = new ofstream ("simulator.pos");
}

SimulatorVision::~SimulatorVision () throw () {
  if (logtruestream) {
    (*logtruestream) << flush;
    delete logtruestream;
  }
}

void SimulatorVision::process_images () throw () {
  the_sim_client->update();
  const FieldGeometry& fg (MWM.get_field_geometry());
  VisibleObjectList vol;
  vol.timestamp = the_sim_client->timestamp;

  // Ball eintragen:
  if (!brandom (ball_mis_probability) && (the_sim_client->ball_position-the_sim_client->robot_position).length()<ball_vision_radius)
    vol.objectlist.push_back (VisibleObject ((the_sim_client->ball_position-the_sim_client->robot_position).rotate(-the_sim_client->robot_heading)+noise_level*n2random(), VisibleObject::ball));

  // Hindernisse eintragen:
  std::vector<Vec>::iterator oit = the_sim_client->obstacle_positions.begin();
  std::vector<Vec>::iterator oit_end = the_sim_client->obstacle_positions.end();
  while (oit<oit_end) {
    if (!brandom (obstacle_mis_probability) && ((*oit)-the_sim_client->robot_position).length()<obstacle_vision_radius) {
      Vec opos = ((*oit)-the_sim_client->robot_position).rotate(-the_sim_client->robot_heading)+noise_level*n2random();
      double len = opos.length();
      if (len>200)
        opos*=(1.0-150.0/len);  // Hindernis etwas zum Betrachter herruecken
      vol.objectlist.push_back (VisibleObject (opos, VisibleObject::obstacle, 300));  // Hindernisbreite pauschal mit 300mm angenommen
    }
    oit++;
  }

  // Tore eintragen:
  Vec blue_goal (0,-0.5*fg.field_length-fg.goal_length);
  if (!brandom (goal_mis_probability))
    vol.objectlist.push_back (VisibleObject ((blue_goal-the_sim_client->robot_position).rotate(-the_sim_client->robot_heading)+noise_level*n2random(), VisibleObject::blue_goal));
  Vec yellow_goal (0, 0.5*fg.field_length+fg.goal_length);
  if (!brandom (goal_mis_probability))
    vol.objectlist.push_back (VisibleObject ((yellow_goal-the_sim_client->robot_position).rotate(-the_sim_client->robot_heading)+noise_level*n2random(), VisibleObject::yellow_goal));

  // Linien eintragen:
  Vec robotpos = -the_sim_client->robot_position;
  Angle robothead = the_sim_client->robot_heading+Angle::half;;
  try{
    vector<Line>::iterator bit = scanlines.begin();
    vector<Line>::iterator bit_end = scanlines.end();
    vector<LineSegment>::iterator lit;
    vector<LineSegment>::iterator lit_end = lines.end();
    vector<Arc>::iterator ait;
    vector<Arc>::iterator ait_end = arcs.end();
    double lvr2 = line_vision_radius*line_vision_radius;
    while (bit<bit_end) {
      lit=lines.begin();
      while (lit<lit_end) {
        vector<Vec> pvec = intersect (bit->translate (robotpos), *lit);
        for (unsigned int i=0; i<pvec.size(); i++) {
          if (((pvec[i]-robotpos).squared_length()<lvr2) && !brandom (line_mis_probability)) {
            vol.objectlist.push_back (VisibleObject ((pvec[i]-robotpos).rotate(-robothead)+noise_level*n2random(), VisibleObject::white_line));
          }
        }
        lit++;
      }
      ait=arcs.begin();
      while (ait<ait_end) {
        try{
          vector<Vec> p = intersect (bit->translate (robotpos), *ait);
          for (unsigned int i=0; i<p.size(); i++)
            if ((p[i]-robotpos).squared_length()<lvr2)
              if (!brandom (line_mis_probability))
                vol.objectlist.push_back (VisibleObject ((p[i]-robotpos).rotate(-robothead)+noise_level*n2random(), VisibleObject::white_line));
        }catch(exception&){;}  // kein Schnittpunkt oder bad_alloc
        ait++;
      }
      bit++;
    }
  }catch(std::bad_alloc){;}  // ignorieren
  MWM.set_visual_information (vol, 0); // camera 0
  const RobotLocation rloc = MWM.get_robot_location (vol.timestamp);
  if (MWM.get_own_half ()<0) {
    robotpos*=-1;
    robothead+=Angle::half;
  }
//  if ((rloc.pos-robotpos).length()>200 || (rloc.heading.in_between (robothead+Angle::deg_angle(10), robothead-Angle::deg_angle(10))))
//    MWM.reset (robotpos, robothead);

  // Gyroskop:
  if (simulate_gyro) {
    gyro_phi_noise+=0.0001*nrandom();  // Drift des Gyros mitmodellieren
    bound (gyro_phi_noise, -0.02, 0.02);
    GyroData gd;
    gd.vrot=the_sim_client->robot_angular_velocity+gyro_phi_noise;
    MWM.set_gyro_data (gd, the_sim_client->timestamp);
  }
  if (simulate_odo) {
    odo_phi_noise+=0.01*nrandom();
    odo_x_noise+=0.01*nrandom();
    odo_y_noise+=0.01*nrandom();
    bound (odo_phi_noise, 0.7, 1.3);
    bound (odo_x_noise, 0.7, 1.3);
    bound (odo_y_noise, 0.7, 1.3);
    DriveVector odo;
    odo.kick=false;
    odo.vrot=the_sim_client->robot_angular_velocity*odo_phi_noise;
    odo.vtrans=the_sim_client->robot_linear_velocity.rotate (-the_sim_client->robot_heading);
    odo.vtrans.x*=odo_x_noise;
    odo.vtrans.y*=odo_y_noise;
    MWM.set_odometry (odo, the_sim_client->timestamp);
  }

  if (logtruestream) {
    (*logtruestream) << the_sim_client->timestamp.get_msec() << '\t'
        << -the_sim_client->robot_position << '\t'
        << (the_sim_client->robot_heading+Angle::half).get_rad() << '\t'
        << -the_sim_client->ball_position << '\n';
  }
}
