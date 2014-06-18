
#include "ErrorMinimiserWorldModel.h"
#include "../SL/TemporalDifferenceSL.h"
#include "../WorldModelFactory.h"
#include "../WorldModel.h"
#include "../Prediction/update_robot_location.h"
#include "../../Fundamental/stringconvert.h"
#include "../../Structures/Journal.h"

#define INCLUDE_PFILTER 0
#define USE_ALL_CAMERA_LINES 0
#define INCLUDE_ERROR_MINIMISER 0

#if INCLUDE_PFILTER
#include "../SL/CondensationFilter.h"
#endif
#if INCLUDE_ERROT_MINIMISER
#include "../SL/ErrorMinimiserSL.h"
#endif

using namespace std;
using namespace Tribots;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("ErrorMinimiser"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType*) throw (TribotsException,bad_alloc) {
      return new ErrorMinimiserWorldModel(reader);
    }
  };
  Builder the_builder;
}



ErrorMinimiserWorldModel::ErrorMinimiserWorldModel (const ConfigReader& reader) throw (std::bad_alloc) : WorldModelTypeBase (reader), ball_filter (reader,odobox), obstacle_filter (reader, get_field_geometry()) {
  std::string ftype;
  reader.get ("WorldModel::sl_filter_type", ftype);
  if (ftype=="temporal_difference")
    sl = new TemporalDifferenceSL (reader, odobox, get_field_geometry());
#if INCLUDE_PFILTER
  else if (ftype=="condensation_filter")
    sl = new CondensationFilter (reader, odobox, get_field_geometry());
#endif
#if INCLUDE_ERROR_MINIMISER
  else if (ftype=="error_minimiser")
    sl = new ErrorMinimiserSL (reader, odobox, get_field_geometry());
#endif
  else
    sl = new TemporalDifferenceSL (reader, odobox, get_field_geometry());

  usec_sl=usec_ball=usec_obstacle=num_cycle=0;
  report_computation_time=report_computation_time_per_cycle=false;
  reader.get ("WorldModel::report_computation_time", report_computation_time);
  reader.get ("WorldModel::report_computation_time_per_cycle", report_computation_time_per_cycle);
}

ErrorMinimiserWorldModel::~ErrorMinimiserWorldModel () throw () {
  if (report_computation_time) {
    stringstream inout;
    inout << "average SL-time was " << usec_sl/num_cycle << " usec\n";
    inout << "average ballfilter time was " << usec_ball/num_cycle << " usec\n";
    inout << "average obstacle filter time was " << usec_obstacle/num_cycle << " usec" << endl;
    string line;
    getline (inout, line);
    JMESSAGE (line.c_str());
    getline (inout, line);
    JMESSAGE (line.c_str());
    getline (inout, line);
    JMESSAGE (line.c_str());
  }
  delete sl;
}

void ErrorMinimiserWorldModel::reset () throw () {
  sl->reset ();
}

void ErrorMinimiserWorldModel::reset (const Vec p) throw () {
  sl->reset (get_own_half()*p);
}

void ErrorMinimiserWorldModel::reset (const Vec p, const Angle h) throw () {
  sl->reset (get_own_half()*p,h+(get_own_half()==-1? Angle::half : Angle::zero));
}

void ErrorMinimiserWorldModel::slMirrorHint (Vec v) throw () {
  sl->slMirrorHint (get_own_half()*v);
}

unsigned int ErrorMinimiserWorldModel::update_localisation () throw () {
  unsigned int ret=0;
  Time mtime;
#if USE_ALL_CAMERA_LINES
  // SL aktualisieren (Linien von allen Kameras, Zeitstempel von Kamera 0)
  VisibleObjectList vol = visbox[0].get_lines();
  for (unsigned int i=1; i<visbox.size(); i++)
    vol.objectlist.insert (vol.objectlist.end(), visbox[i].get_lines().objectlist.begin(), visbox[i].get_lines().objectlist.end());
  if (sl->update (vol, visbox[0].get_obstacles(), visbox[0].get_goals())) {
    latest_slupdate_timestamp = vol.timestamp;
    ret |= 1;
  }
#else
  // evtl. Zeitstempel korrigieren, da Zeitstempel bei verzoegerten Zyklen nicht passen
  bool pcycle_delayed = (visbox[0].get_timestamp().diff_msec (latest_vis_timestamp)>100);
  if (cycle_delayed) {  // War der letzte Zyklus verzoegert, ist der jetzige nicht verzoegert (Annahme: keine zwei verzoegerten Zyklen in Folge)
    latest_vis_timestamp = visbox[0].get_timestamp();
    cycle_delayed=false;
  } else {
    if (pcycle_delayed) { // verzoegerter Zyklus. Diese Information passt eigentlich noch in den normalen Takt, erst im naechsten Zyklus kommen die wirklich verzoegerten Informationen an
      cycle_delayed=true; // fuer den naechsten Zyklus merken, dass eine Verzoegerung vorliegt
      latest_vis_timestamp.add_msec(34); // einen kuenstlichen Zeitstempel erzeugen im normalen Takt (34ms)
      visbox[0].set_timestamp (latest_vis_timestamp);
    } else {
      latest_vis_timestamp = visbox[0].get_timestamp();
    }
  }
  // SL aktualisieren (nur von Kamera 0)
  if (sl->update (visbox[0].get_lines(), visbox[0].get_obstacles(), visbox[0].get_goals())) {
    latest_slupdate_timestamp = visbox[0].get_timestamp();
    ret |= 1;
  }
#endif
  unsigned long int sltime = mtime.elapsed_usec();

  // Ball aktualisieren
  // die verschiedenen Kameraquellen in zeitlicher Reihenfolge abarbeiten:
  vector<VisibleObjectList> ball_vis (visbox.size());
  vector<RobotLocation> rpos_tvis (visbox.size());
  for (unsigned int i=0; i<visbox.size(); i++) {
    ball_vis[i] = visbox[i].get_balls();
    rpos_tvis[i] = flip_sides (sl->get (ball_vis[i].timestamp), get_own_half());
  }
  if (ball_filter.update (ball_vis, rpos_tvis))
    ret |= 2;
  unsigned long int balltime = mtime.elapsed_usec();

  // Hindernisse aktualisieren (nur von Kamera 0)
  RobotLocation pos_tvis = flip_sides (sl->get (latest_slupdate_timestamp), get_own_half());
  BallLocation bloc_tvis = ball_filter.get (latest_slupdate_timestamp);
  obstacle_filter.update (visbox[0].get_obstacles(), pos_tvis, bloc_tvis);
  ret |= 4;  // Bei Hindernissen kann nicht zwischen Erfolg/Misserfolg unterschieden werden, es koennen ja tatsaechlich keine gesehen worden sein
  unsigned long int obstacletime = mtime.elapsed_usec();

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

  if (report_computation_time) {
    num_cycle++;
    usec_sl+=sltime;
    usec_ball+=(balltime-sltime);
    usec_obstacle+=(obstacletime-balltime);
  }
  if (report_computation_time_per_cycle) {
    LOUT << "WorldModelTimes SL/Ball/Obstacle [usec]: " << sltime << ' ' << balltime-sltime << ' ' << obstacletime-balltime << '\n';
  }

  return ret;
}

RobotLocation ErrorMinimiserWorldModel::get_robot (Time t) const throw () {
  RobotLocation dest = flip_sides (sl->get (t), get_own_half());
  dest.stuck = stuck_sensor.get_stuck_location (dest.pos, dest.vtrans);
  return dest;
}

BallLocation ErrorMinimiserWorldModel::get_ball (Time t) const throw () {
  return ball_filter.get (t);
}

ObstacleLocation ErrorMinimiserWorldModel::get_obstacles (Time) const throw () {
  return obstacle_filter.get_with_poles_and_stuck ();
}

Time ErrorMinimiserWorldModel::get_timestamp_latest_update () const throw () { 
  return latest_slupdate_timestamp;
}
