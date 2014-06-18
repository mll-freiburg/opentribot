
#include "WorldModelTypeBase.h"
#include "../../Structures/GameState.h"
#include "../Prediction/LocationShortTermMemory.h"
#include "../Prediction/SingleStepHeuristicVelocityPredictor.h"
#include "../FataMorgana.h"
#include "../WorldModel.h"
#include <cmath>
#include <iomanip>

using namespace Tribots;
using namespace std;

#define DEBUG_SLVEL 0

Tribots::WorldModelTypeBase::WorldModelTypeBase (const ConfigReader& vread) throw (InvalidConfigurationException, std::bad_alloc) : field_geometry (vread), gsman (vread, field_geometry), own_half (1), odobox (20, 5, 10), locations (new LocationShortTermMemory (*this)), velocity_predictor (new SingleStepHeuristicVelocityPredictor (vread, odobox)), null_stream ("/dev/null"), tournament_mode (true), velocity_filter (5), stuck_sensor (vread), robot_id (0), prediction_error_out (NULL), prediction_testdelay(0), old_predictions (20) {
  string confline;
  if (vread.get ("own_half", confline)>0) {
    if (confline=="yellow")
      own_half=1;
    else if (confline=="blue")
      own_half=-1;
  }
  
cout <<"Hello"<<  field_geometry.field_length<<endl;

  null_stream.close();   // Zur absoluten Sicherheit, dass nicht rausgeschrieben werden kann bzw. Betriebssystem nicht eingreifen muss
  even_cycle=0;
  VisualContainer vc;
  visbox.push_back (vc); // visbox enthaelt mindestens einen VisualContainer
  vread.get ("tournament_mode", tournament_mode);
  bool prediction_test = false;
  vread.get ("WorldModel::velocity_prediction_test", prediction_test);
  if (prediction_test) {
    prediction_error_out = new ofstream ("velocity_prediction_test.dat");
    (*prediction_error_out) << setprecision (4);
    vread.get ("WorldModel::velocity_prediction_delay", prediction_testdelay);
  }
}

Tribots::WorldModelTypeBase::~WorldModelTypeBase () throw () {
  delete locations;
  if (velocity_predictor) delete velocity_predictor;
  if (prediction_error_out) {
    prediction_error_out->flush();
    delete prediction_error_out;
  }
}


// ------------------ Anfragen und Veraenderung von Zustaenden: --------------------

const Tribots::FieldGeometry& Tribots::WorldModelTypeBase::get_field_geometry () const throw () { return field_geometry; }

int Tribots::WorldModelTypeBase::get_own_half () const throw () { return own_half; }

const GameState& Tribots::WorldModelTypeBase::get_game_state () const throw () { return gsman.get(); }

const Tribots::RobotProperties& Tribots::WorldModelTypeBase::get_robot_properties () const throw () { return robot_properties; }

void Tribots::WorldModelTypeBase::set_own_half (int h) throw () { own_half = ( h>=0 ? 1 : -1);}

void Tribots::WorldModelTypeBase::set_robot_properties (const Tribots::RobotProperties& rpr) throw () { robot_properties=rpr; }

const Tribots::RobotData& Tribots::WorldModelTypeBase::get_robot_data (Time & _time) const throw () { _time = robot_data_time; return robot_data; };

void Tribots::WorldModelTypeBase::set_robot_data (const Tribots::RobotData& rd, Time _time) throw() { robot_data = rd; robot_data_time = _time;};

std::ostream& Tribots::WorldModelTypeBase::log_stream () throw () { return null_stream; }

void Tribots::WorldModelTypeBase::reset (const Vec p, const Angle) throw () { reset (p); }

void Tribots::WorldModelTypeBase::startstop (bool b) throw () { gsman.set_in_game (b); }

void WorldModelTypeBase::init_cycle (Time t1, Time t2) throw () { gsman.init_cycle (t1, t2); }

void WorldModelTypeBase::set_score (unsigned int own_score, unsigned int opponent_score, unsigned int yellow_cards) throw () {
  gsman.set_score (own_score, opponent_score, yellow_cards);
}


// ---------------- update-Methoden: ----------------

void Tribots::WorldModelTypeBase::update () throw () {
  // Fata Morgana einblenden
  for (unsigned int i=0; i<fata_morgana.size(); i++) {
    fata_morgana[i]->update (this);
  }

  update_game_state ();  // game state aktualisieren
  unsigned int updated=update_localisation ();  // Lokalisierung, Ballfilter etc. aktualisieren
  Time tref = get_timestamp_latest_update ();

  // Geschwindigkeitsschaetzer und Stuck
  if (updated&1) {
    const RobotLocation pos_tref = get_robot (tref);
    velocity_filter.update (pos_tref, tref);
    stuck_sensor.update ();
#if DEBUG_SLVEL
    Time dummy;
    RobotLocation rloc = velocity_filter.get (dummy);
    LOUT << "SLVel: " << rloc.vtrans << rloc.vrot << "  " << rloc.vtrans.length() << '\n';
#endif
  }

  // VelocityPredictor aktualisieren
  if (velocity_predictor) {
    if (updated&1) {
      RobotLocation rloc = get_robot_location (tref, false);
      velocity_predictor->notify_position (rloc, tref);
    }
    velocity_predictor->update ();
  }

  // VelocityPrediction Test:
  if (prediction_error_out) {
    // Praediktion-Ist-Vergleich
    TRL trl;
    trl.rloc = velocity_filter.get (trl.timestamp);
    trl.timestamp.add_msec (-67);  // wegen Zeitstempel des Velocity-Filters ist aktuellste Beobachtung, hier woll aber die Mitte des Zeitfensters genommen werden
    if (trl.timestamp.get_msec ()>2000) {  // Einschwingphase ignorieren (2s) bis Puffer gefuellt
      for (unsigned int i=0; i<old_predictions.size(); i++) {
        if (old_predictions[i].timestamp>trl.timestamp) {
          int delta_t1 = trl.timestamp.diff_msec (old_predictions[i-1].timestamp);
          int delta_t2 = old_predictions[i].timestamp.diff_msec (trl.timestamp);
          double tau = static_cast<double>(delta_t1)/static_cast<double>(delta_t1+delta_t2);
          bool anglemode = (trl.rloc.heading.in_between (Angle::quarter, Angle::three_quarters));
          (*prediction_error_out) << trl.timestamp << ' ' << trl.rloc.pos << ' ' << (anglemode ? trl.rloc.heading.get_deg() : trl.rloc.heading.get_deg_180()) << ' ';
          (*prediction_error_out) << trl.rloc.vtrans << ' ' << trl.rloc.vrot << "   ";
          Angle delta_h1 = Angle::rad_angle(old_predictions[i-1].rloc.vrot*delta_t1);
          Angle delta_h2 = Angle::rad_angle(old_predictions[i].rloc.vrot*delta_t2);
          Vec v1 = old_predictions[i-1].rloc.vtrans*delta_h1;
          Vec v2 = old_predictions[i].rloc.vtrans/delta_h2;
          Angle avg_angle = (tau*Vec::unit_vector(old_predictions[i-1].rloc.heading)+(1-tau)*Vec::unit_vector(old_predictions[i].rloc.heading)).angle();
          (*prediction_error_out) << tau*old_predictions[i-1].rloc.pos+(1-tau)*old_predictions[i].rloc.pos << ' ' << (anglemode ? avg_angle.get_deg() : avg_angle.get_deg_180()) << ' ';
          (*prediction_error_out) << tau*v1+(1-tau)*v2 << ' ' << tau*old_predictions[i-1].rloc.vrot+(1-tau)*old_predictions[i].rloc.vrot << '\n';
          break;
        }
      }
    }
    if (updated&1) {
      // neue Praediktion in den Ringpuffer einfuegen
      trl.timestamp = tref;
      trl.timestamp.add_msec (prediction_testdelay);
      trl.rloc = get_robot_location (trl.timestamp, false);
      old_predictions.get()=trl;
      old_predictions.step();
    }
  }

  // Relativer Ball
  if (visbox.size()>0 && visbox[0].get_balls().objectlist.size()>0) {
    ball_relative.pos = visbox[0].get_balls().objectlist[0].pos;
    ball_relative.valid=true;
  } else {
    ball_relative.valid=false;
  }

  // visbox leeren
  even_cycle = 1-even_cycle;
  for (unsigned int i=0; i<visbox.size(); i++)
    visbox[i].clear();
  for (unsigned int i=0; i<all_visible_objects[even_cycle].size(); i++)
    all_visible_objects[even_cycle][i].objectlist.clear();

  // teammates auf Aktualitaet pruefen
  unsigned int i=0;
  while (i<teammates.size()) {
    if (teammates[i].timestamp.elapsed_msec()>2000)
      teammates.erase (teammates.begin()+i);
    else
      i++;
  }
  locations->clear();
}

void Tribots::WorldModelTypeBase::update_game_state () throw () {
  gsman.update();
}

void Tribots::WorldModelTypeBase::update_refbox (RefboxSignal sig) throw () {
  gsman.update (sig);
}


// ---------------- set-Methoden: ------------------

void WorldModelTypeBase::set_drive_vector (DriveVector dv, Time t) throw () {
  if (t.diff_msec(recent_drive_vector_time)>=100) {
    // zusaetzlichen Nullvector setzen, wenn TMC ausgesetzt hat (mehr als 100 ms Response Time)
    DriveVector dv0 (Vec(0,0),0,false);
    Time t0 = recent_drive_vector_time;
    recent_drive_vector_time.add_msec(1);
    odobox.add_drive_vector (dv0, t0);
  }
  odobox.add_drive_vector (dv, t);
  recent_drive_vector = dv;
//  cout <<"Worldmodel Tecent drive"<<recent_drive_vector.vtrans<<endl;
  if (velocity_predictor) velocity_predictor->notify_drive_vector (dv, t);
  recent_drive_vector_time=t;
}

void WorldModelTypeBase::set_odometry (DriveVector dv, Time t) throw () {
  if (t.diff_msec(recent_odometry_time)>=100) {
    // zusaetzlichen Nullvector setzen, wenn TMC ausgesetzt hat (mehr als 100 ms Response Time)
    DriveVector dv0 (Vec(0,0),0,false);
    Time t0 = recent_odometry_time;
    recent_odometry_time.add_msec(1);
    odobox.add_odometry (dv0, t0);
  }
  odobox.add_odometry (dv, t);
  recent_odometry_time=t;
}

void WorldModelTypeBase::set_gyro_data (GyroData gd, Time t) throw () {
  odobox.add_gyro_data (gd, t);
}

void WorldModelTypeBase::set_visual_information (const VisibleObject& v, Time t, unsigned int c) throw () {
  while (c>=visbox.size()) {
    VisualContainer vc;
    visbox.push_back (vc);
  }
  visbox[c].add (v, t);
  while (c>=all_visible_objects[even_cycle].size()) {
    VisibleObjectList vol;
    all_visible_objects[even_cycle].push_back (vol);
  }
  if (t!=all_visible_objects[even_cycle][c].timestamp) {
    all_visible_objects[even_cycle][c].objectlist.clear();
    all_visible_objects[even_cycle][c].timestamp=t;
  }
  all_visible_objects[even_cycle][c].objectlist.push_back (v);
}

void WorldModelTypeBase::set_teammates (std::vector<TeammateLocation>& tlo) throw () {
  for (unsigned int i=0; i<tlo.size(); i++) {
    unsigned int j=0;
    while (j<teammates.size()) {
      if (teammates[j].number==tlo[i].number)
        break;
      else
        j++;
    }
    if (j==teammates.size())
      teammates.push_back (tlo[i]);
    else
      teammates[j]=tlo[i];
  }
}

void WorldModelTypeBase::set_robot_id (unsigned int id) throw () {
  robot_id = id;
}

// ------------------- get-Methoden: ----------------------

const RobotLocation& WorldModelTypeBase::get_slfilter_robot_location (Time& t) const throw () {
  return velocity_filter.get (t);
}

MessageBoard& WorldModelTypeBase::get_message_board () throw () {
  return mboard;
}

const RobotLocation& WorldModelTypeBase::get_robot_location (Time t, bool b) throw () {
  return locations->get_robot_location (t, b);
}

const BallLocation& WorldModelTypeBase::get_ball_location (Time t, bool b) throw () {
  return locations->get_ball_location (t, b);
}

const ObstacleLocation& WorldModelTypeBase::get_obstacle_location (Time t, bool b) throw () {
  return locations->get_obstacle_location (t, b);
}

const std::vector<VisibleObjectList>& WorldModelTypeBase::get_visible_objects () throw () {
  // nicht die aktuell zu fuellende sondern die Liste aus der letzten Iteration liefern
  // Vorkehrung, falls irgendwann asynchroner Zugriff auf Kameras  
  return all_visible_objects[1-even_cycle];
}

const DriveVector& WorldModelTypeBase::get_recent_drive_vector () const throw () {
  return recent_drive_vector;
}

const std::vector<TeammateLocation>& WorldModelTypeBase::get_teammate_location () const throw () {
  return teammates;
}

unsigned int WorldModelTypeBase::get_robot_id () throw () {
  return robot_id;
}

const BallRelative& WorldModelTypeBase::get_ball_relative () throw () {
  return ball_relative;
}

// --------------- Fata Morgana-Methoden: ----------------

void WorldModelTypeBase::add_fata_morgana (FataMorgana* fm) throw () {
  if (!tournament_mode) fata_morgana.push_back (fm);
}
void WorldModelTypeBase::add_ball_relative (Vec p) throw () {
  VisibleObject vobj (p, VisibleObject::ball, 0);
  this->set_visual_information (vobj, all_visible_objects[even_cycle][0].timestamp, 0);
}
void WorldModelTypeBase::add_ball_absolute (Vec p) throw () {
  RobotLocation rloc = this->get_robot_location (all_visible_objects[even_cycle][0].timestamp, false);
  add_ball_relative ((p-rloc.pos)/rloc.heading);
}
void WorldModelTypeBase::add_obstacle_relative (Vec p, double w) throw () {
  VisibleObject vobj (p, VisibleObject::obstacle, w);
  this->set_visual_information (vobj, all_visible_objects[even_cycle][0].timestamp, 0);
}
void WorldModelTypeBase::add_obstacle_absolute (Vec p, double w) throw () {
  RobotLocation rloc = this->get_robot_location (all_visible_objects[even_cycle][0].timestamp, false);
  add_obstacle_relative ((p-rloc.pos)/rloc.heading, w);
}

void WorldModelTypeBase::set_active_behavior (const char* n) throw () {
  behavior=n;
}

const char* WorldModelTypeBase::get_active_behavior () const throw () {
  return behavior.c_str();
}



