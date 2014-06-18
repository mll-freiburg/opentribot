
#include "SLStuckSensor.h"
#include "../WorldModel.h"
#include "../Prediction/update_robot_location.h"
#include <cmath>

using namespace Tribots;
using namespace std;

#define DEBUG_SF 1


SLStuckSensor::SLStuckSensor (const ConfigReader& cfg) throw (std::bad_alloc) : n(15), oldpos(n), stuck_min (15) {
  cfg.get ("StuckSensor::num_cycles", stuck_min);
  count=0;
  timestamp_latest_stuck.add_sec (-100);
  pos_latest_stuck = Vec ( 1e100, 1e100 );
  dir_latest_stuck = Vec::zero_vector;
  was_stuck=false;
}

SLStuckSensor::~SLStuckSensor () throw () {;}

void SLStuckSensor::update () throw () {
  // SL-Geschwindigkeit abfragen
  Time timestamp;
  Vec slvel = MWM.get_slfilter_robot_location (timestamp).vtrans;
  Vec slpos = MWM.get_slfilter_robot_location (timestamp).pos;
  // DriveVector-Geschwindigkeit abfragen
  unsigned int i=0;
  while (i<n && oldpos[i].timestamp<timestamp) 
    i++;
  Vec dvvel;
  double dvderiv;
  double dvrot;
  if (i>0 && i<n) {
    double tau = static_cast<double>(timestamp.diff_msec (oldpos[i-1].timestamp))/static_cast<double>(oldpos[i].timestamp.diff_msec (oldpos[i-1].timestamp));
    dvvel = tau*oldpos[i-1].rloc.vtrans+(1.0-tau)*oldpos[i].rloc.vtrans;
    dvderiv = tau*oldpos[i-1].deriv+(1-tau)*oldpos[i].deriv;
    dvrot = tau*oldpos[i-1].rloc.vrot+(1-tau)*oldpos[i].rloc.vrot;
  } else if (i>=n) {
    dvvel = oldpos[n-1].rloc.vtrans;
    dvderiv = oldpos[n-1].deriv;
    dvrot = oldpos[n-1].rloc.vrot;
  } else {
    dvvel = oldpos[0].rloc.vtrans;
    dvderiv = oldpos[0].deriv;
    dvrot = oldpos[0].rloc.vrot;
  }

  // Stuck-Kriterium: zu grosser Unterschied zwischen slvel und dvvel bei kleinerm dvderiv
  double c1 = (dvvel*slvel)/(dvvel.length()+0.01);
  double c2 = dvvel.length();
  if ((slvel.length()<0.9) && ((abs(dvrot)<2 && abs(dvderiv)<2 && 3*c1<c2 && 0.5<c2 && abs(c1-c2)>0.7) || (abs(dvrot)<2 && abs(dvderiv)<1 && 3*c1<c2 && 0.5<c2) )) {
    count++;
    if (count>=stuck_min) {
      pos_latest_stuck = slpos;
      dir_latest_stuck = dvvel;
      timestamp_latest_stuck.update ();
    }
  } else if (slvel.length()>1.1 || abs(dvderiv)<2) {
    count=0;
  }
  if (count>=stuck_min) {
    if ((slpos-pos_latest_stuck).length()>500)
      count=0;       // wenn Roboter weit genug von Stuck-Position entfernt, kein stuck mehr
    if ((slpos-pos_latest_stuck).length()>300 && dir_latest_stuck*(slpos-pos_latest_stuck)<=0)
      count=0;      // wenn Roboter von Stuck-Position weg und in andere Richtung faehrt, kein stuck mehr
  }

#if DEBUG_SF
  Time now;
  LOUT << "Stuckinfo: " << now << ' '
       << dvvel.length() << ' '
       << slvel.length() << ' '
       << (dvvel*slvel)/(dvvel.length()+0.01) << ' '
       << dvderiv << '\n';
  LOUT << "StuckCriterion: " << now << ' ' << count << ' ' << ((abs(dvrot)<2 && abs(dvderiv)<2 && abs(c1-c2)>0.7) || (abs(dvrot)<2 && abs(dvderiv)<1) ? 1 : 0) << '\n';
  LOUT << "\% dark_red cross " << pos_latest_stuck.x << ' ' << pos_latest_stuck.y << '\n';
  if ((abs(dvrot)<2 && abs(dvderiv)<2 && abs(c1-c2)>0.7) || (abs(dvrot)<2 && abs(dvderiv)<1))
    LOUT << "\% orange thick solid circle 0 0 50\n";
  if (count>=1) LOUT << "\% red thick solid circle 0 0 100\n";
  if (count>=2) LOUT << "\% red thick solid circle 0 0 150\n";
  if (count>=4) LOUT << "\% red thick solid circle 0 0 200\n";
  if (count>=6) LOUT << "\% red thick solid circle 0 0 250\n";
  if (count>=8) LOUT << "\% red thick solid circle 0 0 300\n";
  if (count>=10) LOUT << "\% red thick solid circle 0 0 350\n";
  if (count>=12) LOUT << "\% red thick solid circle 0 0 400\n";
  if (count>=14) LOUT << "\% red thick solid circle 0 0 450\n";
  if (count>=stuck_min) LOUT << "\% orange thick solid circle 0 0 500\n";
#endif

  // Neue Position in den Ringpuffer uebernehmen
  TPOS npos;
  npos.timestamp.add_msec (10);
  npos.timestamp.add_msec (static_cast<int>(MWM.get_game_state().actual_cycle_time));
  npos.rloc = MWM.get_robot_location (npos.timestamp, false);
  npos.deriv=0;
  for (unsigned int i=0; i<n; i++) {
    double dt = static_cast<double>(npos.timestamp.diff_msec (oldpos[i].timestamp));
    if (dt<1)
      continue;
    Vec delta_v = npos.rloc.vtrans.rotate (-npos.rloc.heading)-oldpos[i].rloc.vtrans.rotate (-oldpos[i].rloc.heading);
    npos.deriv+=delta_v.length()/dt;
  }
  npos.deriv*=1e3/static_cast<double>(n);
  oldpos.get () = npos;
  oldpos.step();

  // was_stuck aktualisieren
  was_stuck = (count>=stuck_min || timestamp_latest_stuck.elapsed_msec() <=700) || (was_stuck &&  timestamp_latest_stuck.elapsed_msec() <=5000 && (pos_latest_stuck-slpos).squared_length()<250000 && (slvel.angle ()-(pos_latest_stuck-slpos).angle()).in_between (Angle::seven_eighth, Angle::eighth));
}

RobotStuckLocation SLStuckSensor::get_stuck_location (Vec pos, Vec vtrans) const throw () {
  RobotStuckLocation dest;
  dest.msec_since_stuck = timestamp_latest_stuck.elapsed_msec();
  dest.robot_stuck = (count>=stuck_min || dest.msec_since_stuck<=700);
  dest.robot_stuck |= (was_stuck && dest.msec_since_stuck<=5000 && (pos_latest_stuck-pos).squared_length()<250000 && (vtrans.angle ()-(pos_latest_stuck-pos).angle()).in_between (Angle::seven_eighth, Angle::eighth));
  dest.pos_of_stuck = pos_latest_stuck;
  dest.dir_of_stuck = dir_latest_stuck;
  return dest;
}
