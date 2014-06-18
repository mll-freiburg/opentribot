
#include "SPhysGotoPosViaTurningPoint.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include <cmath>

using namespace Tribots;
using namespace std;


SPhysGotoPosViaTurningPoint::SPhysGotoPosViaTurningPoint () throw () : Skill ("SPhysGotoPosViaTurningPoint") {;}

void SPhysGotoPosViaTurningPoint::set_dynamics (double d1, double d2, double d3, double d4) throw () {
  goto_pos_skill.set_dynamics (d1, d2, d3, d4);
}

void SPhysGotoPosViaTurningPoint::set_dynamics (double d1, double d2) throw () {
  goto_pos_skill.set_dynamics (d1, d2);
}

void SPhysGotoPosViaTurningPoint::init (Vec tp, Vec th, Vec tup, double tur, bool st, bool tolt, bool tolr) throw () {
  init (tp, th.angle()-Angle::quarter, tup, tur, st, tolt, tolr);
}

void SPhysGotoPosViaTurningPoint::init (Vec tp, Angle th, Vec tup, double tur, bool st, bool tolt, bool tolr) throw () {
  target_pos = tp;
  target_heading = th;
  turning_point = tup;
  turning_radius = tur;
  do_stop = st;
  tolerance_pos = tolt;
  tolerance_heading = tolr;
}

void SPhysGotoPosViaTurningPoint::gainControl(const Time& t) throw() {
  loseControl(t);
}

void SPhysGotoPosViaTurningPoint::loseControl(const Time&) throw() {
  phase=0;
}

DriveVector SPhysGotoPosViaTurningPoint::getCmd(const Time& t) throw() {
  // zunaechst die Bewegungsphase pruefen und die Tangentenpunkte berechnen
  RobotLocation robot (MWM.get_robot_location (t));
  if (phase==0) {
    // dir und tangent_point2 berechnen
    Line start_target (robot.pos, target_pos);
    dir = start_target.side (turning_point);
    Circle turning_circle (turning_point, turning_radius);
    try{
      vector<Vec> tps = tangent_point (turning_circle, target_pos);
      if (tps.size()==0) { // darf nicht vorkommen 
      } else if (tps.size()==1) {
        tangent_point2=tps[0];
      } else { // 2 tangentiale Punkte
        Angle ttp1 = (tps[0]-target_pos).angle();
        Angle ttp2 = (tps[1]-target_pos).angle();
        Angle ttp = (turning_point-target_pos).angle();
        tangent_point2 = (ttp.in_between (ttp1, ttp2) ? tps[(1+dir)/2] : tps[(1-dir)/2]);
      }
      phase=1;
    }catch(std::exception&) {
      phase=3;  // Zielpunkt liegt im Inneren des Wendekreises, dann fahre direkt dorthin
    }
  }
  if (phase==1) {
    // tangent_point1 berechnen und Uebergang zu Phase 2
    if ((robot.pos-turning_point).length()<turning_radius+200)
      phase=2;
    Circle turning_circle (turning_point, turning_radius);
    try{
      vector<Vec> tps = tangent_point (turning_circle, robot.pos);
      if (tps.size()==0) { // darf nicht vorkommen 
      } else if (tps.size()==1) {
        tangent_point1=tps[0];
      } else { // 2 tangentiale Punkte
        Angle ttp1 = (tps[0]-robot.pos).angle();
        Angle ttp2 = (tps[1]-robot.pos).angle();
        Angle ttp = (turning_point-robot.pos).angle();
        tangent_point1 = (ttp.in_between (ttp1, ttp2) ? tps[(1-dir)/2] : tps[(1+dir)/2]);
      }
    }catch(std::exception&) {
      phase=2;  // augenblickliche Position liegt im Inneren des Wendekreises, dann gehe zu Phase 2 ueber
    }
  }
  if (phase==2) {
    // Uebergang zu Phase 3 berechnen
    if ((robot.pos-tangent_point2).length()<200)
      phase=3;
  }

  //  LOUT << "Phase " << phase << "; Dir " << dir << '\n';
  //  LOUT << "\% dark_green cross " << tangent_point1 << " green cross " << tangent_point2 << " light_green " << target_pos << '\n';

  // jetzt die Bewegung in der jeweiligen Phase berechnen
  if (phase==1) {
    // direkt zum ersten Tangentenpunkt fahren
    goto_pos_skill.init (tangent_point1, target_heading, true);
    return goto_pos_skill.getCmd (t);
  } else if (phase==2) {
    // um den Wendepunkt Kurve fahren
    Vec centerrobot = robot.pos-turning_point;
    Angle current_direction = centerrobot.angle();
    Angle target_direction = (tangent_point2-turning_point).angle();

    double delta_angle = (target_direction-current_direction).get_rad_pi();
    bool far_away = (abs(delta_angle)>0.2);  // Winkeldifferenzen ueber 18 Grad

    Angle new_direction = (far_away ? current_direction+dir*Angle::rad_angle(0.2) : target_direction);
    Vec new_pos = turning_point+turning_radius*Vec::unit_vector(new_direction);
  
    goto_pos_skill.init (new_pos, target_heading, true, false, false);
    return goto_pos_skill.getCmd (t);
  } else {
    // zum Zielpunkt fahren
    goto_pos_skill.init (target_pos, target_heading, do_stop, true, true);
    return goto_pos_skill.getCmd (t);
  }
}


