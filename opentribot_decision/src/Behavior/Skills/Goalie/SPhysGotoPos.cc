
#include "SPhysGotoPos.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

#define DEBUG_GP 1

SPhysGotoPos::SPhysGotoPos () throw () : Skill ("SPhysGotoPos") {
  recent_vrot=0;
  const RobotProperties& rp (MWM.get_robot_properties());

  cout <<" rp maxv " <<rp.max_velocity<<" rp maxrv "<<rp.max_rotational_velocity<<" rp maxacc "<<rp.max_acceleration<<"rp maxracc"<<rp.max_rotational_acceleration<<endl;
  set_dynamics (rp.max_velocity, rp.max_rotational_velocity, rp.max_acceleration, rp.max_rotational_acceleration);
  init (Vec::zero_vector, 0, true);
}

SPhysGotoPos::~SPhysGotoPos () throw () {;}

void SPhysGotoPos::init (Vec tp, Vec th, bool st, bool tolp, bool tolh) throw () {
  init (tp, th.angle()-Angle::quarter, st, tolp, tolh);
}

void SPhysGotoPos::init (Vec tp, Angle ta, bool st, bool tolp, bool tolh) throw () {
  max_target_velocity = (st ? 0.0 : max_tv);
  target_pos = tp;
  target_heading = ta;
  tolerance_pos = tolp;
  tolerance_heading = tolh;
  barrier.clear();
}

void SPhysGotoPos::init (Vec tp, Vec th, double mtv, bool tolp, bool tolh) throw () {
  init (tp, th.angle()-Angle::quarter, mtv, tolp, tolh);
}

void SPhysGotoPos::init (Vec tp, Angle ta, double mtv, bool tolp, bool tolh) throw () {
  max_target_velocity = (mtv>max_tv ? max_tv : mtv);
  target_pos = tp;
  target_heading = ta;
  tolerance_pos = tolp;
  tolerance_heading = tolh;
  barrier.clear();
}

void SPhysGotoPos::init_barrier (const Line& b) throw () {
  barrier.push_back (b);
}

void SPhysGotoPos::init_barrier (const std::vector<Line>& b) throw () {
  barrier.insert (barrier.end(), b.begin(), b.end());
}

void SPhysGotoPos::set_dynamics (double vt) throw () {
  max_tv = vt;
}

void SPhysGotoPos::set_dynamics (double vt, double vr) throw () {
  max_tv = vt;
  max_rv = vr;
}

void SPhysGotoPos::set_dynamics (double vt, double vr, double at, double ar) throw () {
  max_tv = vt;
  max_rv = vr;
  max_ta = at;
  max_ra = ar;
  max_td = 0.75*max_ta;
  max_rd = 0.75*max_ra;
}

void SPhysGotoPos::get_dynamics (double& vt, double& vr, double& at, double& ar, double& bt, double& br) throw () {
  vt = max_tv;
  vr = max_rv;
  at = max_ta;
  ar = max_ra;
  bt = max_td;
  br = max_rd;
}

bool SPhysGotoPos::heading_reached (Time t) const throw () {
  const RobotLocation& rloc_exec = MWM.get_robot_location(t);
  double diff_heading = (target_heading-rloc_exec.heading).get_rad_pi();
  bool heading_okay = false;
  if (abs(diff_heading)<0.06) heading_okay=true;  // minimale Regeldifferenzen (<3.6Grad) ignorieren
  if (abs(diff_heading)<0.12 && abs(rloc_exec.vrot)<1e-2) heading_okay=true;  // wegen kleiner Regeldifferenzen (<7.2Grad) den Roboter nicht in Bewegung setzen
  return heading_okay;
}

bool SPhysGotoPos::position_reached (Time t) const throw () {
  const RobotLocation& rloc_exec = MWM.get_robot_location(t);
  Vec desired_dir = target_pos-rloc_exec.pos;
  bool position_okay=false;
  if (desired_dir.length()<50) position_okay=true;  // minimale Regeldifferenzen (<5cm) ignorieren
  if (desired_dir.length()<100 && rloc_exec.vtrans.length()<1e-2) position_okay=true;  // wegen kleiner Regeldifferenzen (<10cm) den Roboter nicht in Bewegung setzen
  return position_okay;
}

bool SPhysGotoPos::destination_reached (Time t) const throw () {
  return position_reached(t) && heading_reached(t);
}

DriveVector SPhysGotoPos::getCmd(const Time& texec) throw() {
  DriveVector dest;
  Time texec2 (texec);
  texec2.add_msec (120);
  const RobotLocation& rloc_exec = MWM.get_robot_location(texec2);

  // Debug: Ziel, Ausrichtung und Barrieren einzeichnen
#if DEBUG_GP
  LOUT << "% red solid thin Tarrow " << target_pos << ' ' << target_pos+(300*max_target_velocity+200)*Vec::unit_vector(target_heading+Angle::quarter) << " black dashed \n";
  for (unsigned int i=0; i<barrier.size(); i++) {
    Vec pp = barrier[i].perpendicular_point (target_pos);
    Vec dir = (barrier[i].p1-barrier[i].p2).normalize();
    LOUT << "% line " << pp-1500*dir << ' ' << pp+1500*dir << '\n';
  }
#endif
  
  // Rotation ausregeln
  double diff_heading = (target_heading-rloc_exec.heading).get_rad_pi();
  if (tolerance_heading) {
    if (abs(diff_heading)<0.06) diff_heading=0;  // minimale Regeldifferenzen (<3.6Grad) ignorieren
    if (abs(diff_heading)<0.12 && abs(rloc_exec.vrot)<1e-2) diff_heading=0;  // wegen kleiner Regeldifferenzen (<7.2Grad) den Roboter nicht in Bewegung setzen
  }
  if (abs(diff_heading)<=0.001)
    dest.vrot=0;
  else if (diff_heading>0) {
    if (diff_heading<0.15)
      dest.vrot = 0.5;
    else if (diff_heading<0.3)
      dest.vrot = 1.5;
    else if (diff_heading<0.6)
      dest.vrot = 3.0;
    else
      dest.vrot = max_rv;
  } else {
    if (diff_heading>-0.15)
      dest.vrot = -0.5;
    else if (diff_heading>-0.3)
      dest.vrot = -1.5;
    else if (diff_heading>-0.6)
      dest.vrot = -3.0;
    else
      dest.vrot = -max_rv;
  }
   


  // Bremsweg fuer Rotation beruecksichtigen
  double m = sqrt (2*max_rd*abs(diff_heading));  // Bremswegformel
  //cout <<"max_rd"<<max_rd<<endl;
  if (dest.vrot<-m) dest.vrot=-m;
  if (dest.vrot>m) dest.vrot=m;

  // Zittern unterdruecken
  if (abs(diff_heading)<0.18) {
    dest.vrot = 0.5*dest.vrot+0.5*recent_vrot;
  }
  recent_vrot=dest.vrot;
  
  // Translation ausregeln
  Vec desired_dir = target_pos-rloc_exec.pos;
  if (tolerance_pos) {
    if (desired_dir.length()<50) desired_dir=Vec::zero_vector;  // minimale Regeldifferenzen (<5cm) ignorieren
    if (desired_dir.length()<100 && rloc_exec.vtrans.length()<1e-2) desired_dir=Vec::zero_vector;  // wegen kleiner Regeldifferenzen (<10cm) den Roboter nicht in Bewegung setzen
  }
  double desired_velocity = 0;
  double way_len = desired_dir.length();

  if (way_len>0) {
    desired_velocity = max_tv;  // Achtung: hier noch in Weltkoordinaten!
    desired_dir/=way_len;
  }
  dest.vtrans =desired_velocity*desired_dir;

 //  cout << "Dest vtrans"<<dest.vtrans<<endl;

  // Maximalbeschleunigung beruecksichtigen, nehme 6 m/s^2 als maximal an
  DriveVector recent_dv = MWM.get_recent_drive_vector ();
  //cout << "recent _dv"<<recent_dv.vtrans<<endl;

  recent_dv.vtrans*=rloc_exec.heading;  // Fahrtvektor in Roboterkoordinaten, daher umrechnen
  double desired_acc = 1e3*(dest.vtrans-recent_dv.vtrans).length()/33;
  if (desired_acc>6) {
    dest.vtrans = recent_dv.vtrans+(6/desired_acc)*(dest.vtrans-recent_dv.vtrans);
  }

//  cout << "Dest vtrans2"<<dest.vtrans<<endl;


  // Bremsweg beruecksichtigen
  if (dest.vtrans.length()>max_target_velocity) {
    double m = sqrt (max_target_velocity*max_target_velocity+2e-3*max_td*way_len);  // Bremswegformel
    if (dest.vtrans.length()>m) dest.vtrans=m*dest.vtrans.normalize();
  }
  
  // Barrieren beruecksichtigen
  for (unsigned int i=0; i<barrier.size(); i++) {
    if (barrier[i].side(rloc_exec.pos)*barrier[i].side(target_pos)>0) { // wenn Roboter und Ziel auf der selben Seite der Barriere
      Vec pp = barrier[i].perpendicular_point (rloc_exec.pos);
      Vec pp_dir = (pp-rloc_exec.pos);
      double bw = pp_dir.length();
      if (bw>0)
        pp_dir/=bw;
      if (pp_dir*dest.vtrans>0) {
        // Roboter bewegt sich auf Barriere zu
        double max_pv = sqrt (2e-3*0.5*max_td*bw); // 0.5 als Sicherheitsmarge, da moeglicherweise Bremsverzoegerung auch noch an anderer Stelle benoetigt wird
        double act_pv = dest.vtrans*pp_dir;
        if (max_pv<act_pv) {
          // Roboter zu schnell
          Vec pportho_dir = pp_dir.rotate_quarter();
          dest.vtrans = max_pv*pp_dir+(dest.vtrans*pportho_dir)*pportho_dir;
#if DEBUG_GP
          LOUT << "Barriere beschraenkt Geschwindigkeit\n";
#endif
        }
      }
    }
  }

/*
  // Zusammenhang zwischen Rotation und Translation beruecksichtigen
  Time tt_est;
  RobotLocation rloc_est = MWM.get_slfilter_robot_location (tt_est);
  if (abs(rloc_est.vrot)>=1e-2) {
    Angle half_expected_rotation = Angle::rad_angle (0.5e-3*rloc_est.vrot*MWM.get_game_state().actual_cycle_time);
    dest.vtrans /= (half_expected_rotation);
  }
*/
  
  // Umrechnen von Welt- in Roboterkoordinaten
  dest.vtrans/=rloc_exec.heading;
  
  cout <<"SPhysGotoPos"<<dest.vtrans<<endl;

  return dest;
}
