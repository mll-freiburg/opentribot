#include "stdlib.h"
#include "AddWriteWorldModel.h"
#include "../WorldModelFactory.h"
#include "../Prediction/update_robot_location.h"
#include "../../Structures/Journal.h"
#include <fstream>
#include <iostream>

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("AddWriteWorldModel"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType* wm) throw (TribotsException,bad_alloc) {
      return new AddWriteWorldModel (reader, wm);
    }
  };
  Builder the_builder;
}




AddWriteWorldModel::AddWriteWorldModel (const ConfigReader& reader, WorldModelType* wm) throw () : the_world_model (wm) {
  string fbasename;
  latest_game_state.in_game=false;
  latest_game_state.refstate=errorState;
  latest_timestamp.set_sec(-1000);

  // used to construct timestamp-dependent filename for output
  stringstream filename;
  struct timeval tv1;
  Time ttstart;
  ttstart.set_usec(0);
  ttstart.get (tv1);

  bool rotate_log = true;
  reader.get("rotate_log",rotate_log);

  if (reader.get ("write_world_model_info", fbasename)<=0)
    fbasename = "wminfo";
  string tsarg;
  average_vision_delay=40;
  obtained_visual_information=false;

  filename << fbasename;

  if (rotate_log) {
    struct tm* mytm;
    mytm = localtime(&tv1.tv_sec);
    stringstream timestring;
    timestring << (1900+mytm->tm_year) << "-";
    if (mytm->tm_mon < 9) timestring << "0";
    timestring << mytm->tm_mon+1 << "-";
    if (mytm->tm_mday < 10) timestring << "0";
    timestring << mytm->tm_mday << "-";
    if (mytm->tm_hour < 10) timestring << "0";
    timestring << mytm->tm_hour;
    if (mytm->tm_min < 10) timestring << "0";
    timestring << mytm->tm_min;
    if (mytm->tm_sec < 10) timestring << "0";
    timestring << mytm->tm_sec;

    filename << "_" << timestring.str();
  }

  odometry_out = new ofstream ((filename.str()+string(".odo")).c_str());
  drive_vector_out = new ofstream ((filename.str()+string(".drv")).c_str());
  visual_info_out = new ofstream ((filename.str()+string(".vis")).c_str());
  robot_pos_out = new ofstream ((filename.str()+string(".rpos")).c_str());
  ball_pos_out = new ofstream ((filename.str()+string(".bpos")).c_str());
  obs_pos_out = new ofstream ((filename.str()+string(".opos")).c_str());
  log_out = new ofstream ((filename.str()+string(".log")).c_str());
  gs_out = new ofstream ((filename.str()+string(".gs")).c_str());
  rawrobotdata_out = new ofstream ((filename.str()+string(".rrd")).c_str());
  gyrodata_out = new ofstream ((filename.str()+string(".gyro")).c_str());
  messageboard_out = new ofstream ((filename.str()+string(".mbd")).c_str());

  // delete links and update the links
  if (rotate_log)
    for (int i=0;i<11;i++)
      {
        string extension;
        switch(i)
          {
          case 0:
            extension=".odo";
            break;
          case 1:
            extension=".drv";
            break;
          case 2:
            extension=".vis";
            break;
          case 3:
            extension=".rpos";
            break;
          case 4:
            extension=".bpos";
            break;
          case 5:
            extension=".opos";
            break;
          case 6:
            extension=".log";
            break;
          case 7:
            extension=".gs";
            break;
          case 8:
            extension=".rrd";
            break;
          case 9:
            extension=".gyro";
            break;
          case 10:
            extension=".mbd";
            break;
          }
        // now perform the commands
        stringstream deletecmd;
        deletecmd << "rm -f " << fbasename << extension << " >/dev/null 2>&1";
        system(deletecmd.str().c_str());
        // update link to journal.out
        stringstream relinkcmd;
        relinkcmd << "ln -s " << filename.str() << extension << " " << fbasename << extension;
        system(relinkcmd.str().c_str());
      }

  if (!(*odometry_out))
    JWARNING ("error while opening odometry file for writing");
  if (!(*drive_vector_out))
    JWARNING ("error while opening drive_vector file for writing");
  if (!(*visual_info_out))
    JWARNING ("error while opening visual info file for writing");
  if (!(*robot_pos_out))
    JWARNING ("error while opening robot pos file for writing");
  if (!(*ball_pos_out))
    JWARNING ("error while opening ball pos file for writing");
  if (!(*obs_pos_out))
    JWARNING ("error while opening obstacle pos file for writing");
  if (!(*log_out))
    JWARNING ("error while opening log info file for writing");
  if (!(*gs_out))
    JWARNING ("error while opening game state info file for writing");
  if (!(*gyrodata_out))
    JWARNING ("error while opening gyro data info file for writing");
  if (!(*messageboard_out))
    JWARNING ("error while opening message board data info file for writing");
  if (!(*rawrobotdata_out))
    JWARNING ("error while opening raw robot data file for writing");
  else
    (*rawrobotdata_out) << "vis time \t time \t BoardID \t wheel_vel[0,1,2] \t robot_vel[xm,ym,phi] \t aux_robot_vel[xm,ym,phi] \t motor_current[0,1,2] \t motor_output[0,1,2] \t motor_temp_switch[0,1,2] \t motor_temp[0,1,2] \t vcc \t DriveVector (t+1) x y phi\n";

  visual_writer = new VisibleObjectWriter (*visual_info_out);
  drv_writer = new DriveVectorWriter (*drive_vector_out);
  odo_writer = new DriveVectorWriter (*odometry_out);
  gs_writer = new GameStateWriter (*gs_out);
  robot_writer = new RobotLocationWriter (*robot_pos_out);
  ball_writer = new BallLocationWriter (*ball_pos_out);
  obstacle_writer = new ObstacleLocationWriter (*obs_pos_out);
  gyrodata_writer = new GyroDataWriter (*gyrodata_out);
  messageboard_writer = new MessageBoardWriter (*messageboard_out);

  first_visual=true;

  // Feldgeometrie ins .log-File schreiben
  const FieldGeometry& fg (the_world_model->get_field_geometry());
  log_out->precision (5);
  (*log_out) << "FieldGeometry: " << fg.serialize() << '\n';
  Time nulltime;
  nulltime.set_usec (0);
  timeval tv;
  nulltime.get (tv);
  (*log_out) << "StartingTimeval " << tv.tv_sec << ' ' << tv.tv_usec << '\n';
}

AddWriteWorldModel::~AddWriteWorldModel () throw () {
  gs_writer->write (latest_timestamp.get_msec(), latest_game_state, latest_playertype.c_str(), latest_playerrole.c_str(), behavior.c_str());  // da game states mit einem Zyklus Verzoegerung geschrieben werden

  delete visual_writer;
  delete drv_writer;
  delete odo_writer;
  delete gs_writer;
  delete robot_writer;
  delete ball_writer;
  delete obstacle_writer;
  delete gyrodata_writer;
  delete messageboard_writer;

  (*odometry_out) << flush;
  (*drive_vector_out) << flush;
  (*visual_info_out) << flush;
  (*robot_pos_out) << flush;
  (*ball_pos_out) << flush;
  (*obs_pos_out) << flush;
  (*log_out) << flush;
  (*gs_out) << flush;
  (*rawrobotdata_out) << flush;
  (*gyrodata_out) << flush;
  (*messageboard_out) << flush;

  delete odometry_out;
  delete drive_vector_out;
  delete visual_info_out;
  delete robot_pos_out;
  delete ball_pos_out;
  delete obs_pos_out;
  delete log_out;
  delete gs_out;
  delete rawrobotdata_out;
  delete gyrodata_out;
  delete messageboard_out;

  delete the_world_model;
}


const RobotLocation& AddWriteWorldModel::get_robot_location (Time t, bool b) throw () {
  return the_world_model->get_robot_location(t, b);
}

const BallLocation& AddWriteWorldModel::get_ball_location (Time t, bool b) throw () {
  return the_world_model->get_ball_location(t, b);
}

const ObstacleLocation& AddWriteWorldModel::get_obstacle_location (Time t, bool b) throw () {
  return the_world_model->get_obstacle_location(t, b);
}

void AddWriteWorldModel::set_drive_vector (DriveVector dv, Time t) throw () {
  Time now;
  drv_writer->write (now.get_msec(), t.get_msec(), dv);
  the_world_model->set_drive_vector (dv,t);
}

void AddWriteWorldModel::set_odometry (DriveVector dv, Time t) throw () {
  Time now;
  odo_writer->write (now.get_msec(), t.get_msec(), dv);
  the_world_model->set_odometry (dv,t);
}

void AddWriteWorldModel::set_gyro_data (GyroData gd, Time t) throw () {
  Time now;
  gyrodata_writer->write (now.get_msec(), t.get_msec(), gd);
  the_world_model->set_gyro_data (gd,t);
}

void AddWriteWorldModel::set_visual_information (const VisibleObject& vo, Time t, unsigned int camera) throw () {
  if (first_visual) {
    first_visual_timestamp.update();
    first_visual=false;
    average_vision_delay = 0.95*average_vision_delay+0.05*t.elapsed_msec();
  }
  image_timestamp = t;
  obtained_visual_information=true;

//  RobotLocation rloc = get_robot_location (t, false);
//  Vec p = rloc.pos+vo.pos.rotate (rloc.heading);
//  if (abs(p.y)>400 && p.length()>1500)   // TODO: wieder entfernen! WICHTIG
    visual_writer->write (first_visual_timestamp.get_msec(), t.get_msec(), vo, camera);
  the_world_model->set_visual_information (vo,t,camera);
}

void AddWriteWorldModel::set_robot_data (const RobotData& rd, Time t) throw() 
{ 

  RobotLocation robot_location = the_world_model->get_slfilter_robot_location (t);
  robot_location.vtrans/=robot_location.heading;

  (*rawrobotdata_out) << first_visual_timestamp << '\t' << t
                      << '\t' << rd.BoardID
                      << '\t' << rd.wheel_vel[0] << '\t' << rd.wheel_vel[1] << '\t' << rd.wheel_vel[2]
                      << '\t' << rd.robot_vel[0] << '\t' << rd.robot_vel[1] << '\t' << rd.robot_vel[2]
                      << '\t' << robot_location.vtrans.x << '\t' << robot_location.vtrans.y << '\t' << robot_location.vrot
                      << '\t' << rd.motor_current[0] << '\t' << rd.motor_current[1] << '\t' << rd.motor_current[2]
                      << '\t' << rd.motor_output[0] << '\t' << rd.motor_output[1] << '\t' << rd.motor_output[2]
                      << '\t' << rd.motor_temp_switch[0] << '\t' << rd.motor_temp_switch[1] << '\t' << rd.motor_temp_switch[2]
                      << '\t' << rd.motor_temp[0] << '\t' << rd.motor_temp[1] << '\t' << rd.motor_temp[2]
                      << '\t' << rd.motor_vcc
                      << '\t' << rd.dv_set.vtrans.x << '\t' << rd.dv_set.vtrans.y << '\t' << rd.dv_set.vrot
                      << "\n";
  the_world_model->set_robot_data (rd, t); 
}

void AddWriteWorldModel::update () throw () {
  the_world_model->update();
  first_visual=true;   // um bei Eintreffen der naechsten Bildinformation den Zeitstempel first_visual_timestamp zu setzen
  if (!obtained_visual_information) {
    image_timestamp.update();
    image_timestamp.add_msec(-static_cast<long int>(average_vision_delay));
  }
  obtained_visual_information=false;
  RobotLocation rloc_vis = flip_sides (the_world_model->get_robot_location (image_timestamp, false), the_world_model->get_own_half());
  BallLocation bloc_vis = flip_sides (the_world_model->get_ball_location (image_timestamp, false), the_world_model->get_own_half());
  GameState gs = the_world_model->get_game_state ();
  RobotLocation rloc_exec = flip_sides (the_world_model->get_robot_location (gs.expected_execution_time, true), the_world_model->get_own_half());
  BallLocation bloc_exec = flip_sides (the_world_model->get_ball_location (gs.expected_execution_time, true), the_world_model->get_own_half());
  ObstacleLocation oloc = flip_sides (the_world_model->get_obstacle_location (gs.expected_execution_time, true), the_world_model->get_own_half());

  Time now;
  robot_writer->write (now.get_msec(), image_timestamp.get_msec(), rloc_vis, gs.expected_execution_time.get_msec(),  rloc_exec);
  ball_writer->write (now.get_msec(), image_timestamp.get_msec(), bloc_vis, gs.expected_execution_time.get_msec(),  bloc_exec);
  obstacle_writer->write (now.get_msec(), oloc);
  if (now.diff_sec(latest_timestamp)<900)
    gs_writer->write (latest_timestamp.get_msec(), latest_game_state, latest_playertype.c_str(), latest_playerrole.c_str(), behavior.c_str());
  latest_game_state=gs;
  latest_timestamp=now;
  latest_playertype=playertype;
  latest_playerrole=playerrole;
}

void AddWriteWorldModel::update_log () throw () {
  Time now;
  messageboard_writer->write (now.get_msec(), the_world_model->get_message_board ());
}

void AddWriteWorldModel::init_cycle (Time now, Time exec_time) throw () {
  (*log_out) << "\n%%%%cycle " << now << '\t' << get_game_state().cycle_num+1 << '\n';
  the_world_model->init_cycle(now, exec_time);
}

void AddWriteWorldModel::reset () throw () {
  the_world_model->reset();
}

void AddWriteWorldModel::reset (const Vec p) throw () {
  the_world_model->reset(p);
}

void AddWriteWorldModel::reset (const Vec p, const Angle h) throw () {
  the_world_model->reset(p, h);
}

std::ostream& AddWriteWorldModel::log_stream () throw () {
  return (*log_out);
}

const RobotLocation& AddWriteWorldModel::get_slfilter_robot_location (Time& t) const throw () {
  return the_world_model->get_slfilter_robot_location (t);
}

MessageBoard& AddWriteWorldModel::get_message_board () throw () {
  return the_world_model->get_message_board ();
}

void AddWriteWorldModel::set_active_behavior (const char* n) throw () {
  the_world_model->set_active_behavior (n);
  behavior=n;
}

const char* AddWriteWorldModel::get_active_behavior () const throw () {
  return the_world_model->get_active_behavior ();
}

void AddWriteWorldModel::set_player_type (const char* n) throw () {
  playertype=n;
}

void AddWriteWorldModel::set_player_role (const char* n) throw () {
  playerrole=n;
}
