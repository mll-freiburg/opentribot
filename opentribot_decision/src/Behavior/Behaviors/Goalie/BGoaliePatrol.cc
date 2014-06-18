
#include "BGoaliePatrol.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

BGoaliePatrol::BGoaliePatrol (Vec hp, SPhysGotoPos* sp) throw () 
 : Behavior ("BGoaliePatrol"), goto_obs_skill (sp), home_pos(hp) 
{
  if (sp)
    goto_pos_skill = sp;
  else
    goto_pos_skill = &own_goto_pos_skill;
  patrol_area = Circle (home_pos, 1500);
}

void BGoaliePatrol::gainControl (const Time&) throw(TribotsException) {
  const RobotProperties& rp (MWM.get_robot_properties());
  goto_pos_skill->set_dynamics (1.5, 2.0, 0.6*rp.max_acceleration, 0.6*rp.max_rotational_acceleration);
  goto_obs_skill.set_dynamics (2.0, 4.0, 0.6*rp.max_acceleration, 0.6*rp.max_rotational_acceleration);
}

DriveVector BGoaliePatrol::getCmd(const Time& texec) throw() {
  DriveVector dv;
  if (patrol_area.is_inside (MWM.get_robot_location(texec).pos)) {
    double phase = static_cast<double>(texec.get_msec()%5000);
    goto_pos_skill->init (home_pos, Angle::deg_angle (20*sin(phase*M_PI/2500.0)), true, true, false);
    dv = goto_pos_skill->getCmd (texec);
  } else {
    goto_obs_skill.init (home_pos, Angle::zero, true);
    dv = goto_obs_skill.getCmd (texec);
  }
  dv.kick=false;

  return dv;
}

