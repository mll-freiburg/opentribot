
#include "SPhysTurnAroundPos.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

SPhysTurnAroundPos::SPhysTurnAroundPos () throw () {;}

void SPhysTurnAroundPos::init (Vec ct, Vec th, double tr) throw () {
  init (ct, th.angle()-Angle::quarter, tr);
}

void SPhysTurnAroundPos::init (Vec ct, Angle ta, double tr) throw () {
  center=ct;
  target_heading=ta;
  target_radius=tr;
}

void SPhysTurnAroundPos::set_dynamics (double vt, double vr, double at, double ar) throw () {
  goto_pos_skill.set_dynamics (vt, vr, at, ar);
}

void SPhysTurnAroundPos::set_dynamics (double vt, double vr) throw () {
  goto_pos_skill.set_dynamics (vt, vr);
}

DriveVector SPhysTurnAroundPos::getCmd(const Time& texec) throw() {
  RobotLocation robot = MWM.get_robot_location(texec, false);
  Vec centerrobot = robot.pos-center;
  Angle current_direction = centerrobot.angle();
  Angle target_direction = target_heading-Angle::quarter;

  double delta_angle = (target_direction-current_direction).get_rad_pi();
  int dir = (delta_angle<0 ? -1 : 1);
  bool far_away = (abs(delta_angle)>0.2);  // Winkeldifferenzen ueber 18 Grad

  Angle new_direction = (far_away ? current_direction+dir*Angle::rad_angle(0.2) : target_direction);
  double new_radius = target_radius + (far_away ? 150 : 0);

  Angle new_heading = new_direction+Angle::quarter;
  Vec new_pos = center+new_radius*Vec::unit_vector(new_direction);

  LOUT << "\% red cross " << new_pos << " line " << new_pos << ' ' << new_pos+1000*Vec::unit_vector(new_heading+Angle::quarter) << '\n'; 

  goto_pos_skill.init (new_pos, new_heading, true, false, false);
  return goto_pos_skill.getCmd (texec);
}

