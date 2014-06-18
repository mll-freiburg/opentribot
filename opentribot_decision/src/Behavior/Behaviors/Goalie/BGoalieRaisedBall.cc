
#include "BGoalieRaisedBall.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieRaisedBall::BGoalieRaisedBall () throw () : Behavior ("BGoalieRaisedBall"), oldball(15) {
  const FieldGeometry& fg (MWM.get_field_geometry());
  const RobotProperties& rp (MWM.get_robot_properties());
  goal_center = Vec (0,-0.5*fg.field_length);
  goal_half_width = 0.5*fg.goal_width;
  robot_half_width=0.5*rp.robot_width;
  for (unsigned int i=0; i<6; i++) {
    oldball[i].cycle = 0;
    oldball[i].pos = Vec::zero_vector;
    oldball[i].vel = Vec::zero_vector;
  }
  was_active=false;
}

void BGoalieRaisedBall::cycleCallBack(const Time& t) throw() {
  const BallLocation& ball = MWM.get_ball_location (t);
  oldball.get().cycle = MWM.get_game_state().cycle_num;
  oldball.get().pos = ball.pos.toVec();
  oldball.get().vel = ball.velocity.toVec();
  oldball.step(1);
}

bool BGoalieRaisedBall::checkInvocationCondition (const Time& t) throw () {
  const BallLocation& ball = MWM.get_ball_location (t);
  return (ball.pos_known == BallLocation::raised);
}

bool BGoalieRaisedBall::checkCommitmentCondition (const Time& t) throw () {
  return checkInvocationCondition (t) && ! (was_active && t.diff_msec(time_of_start_raised)>3000); // noch keine 3 Sekunden aktiv
}

void BGoalieRaisedBall::gainControl (const Time& t) throw () {
  was_active=true;
  time_of_start_raised=t;
  target_pos=goal_center;
  if (oldball.get().cycle+oldball.size()-1==MWM.get_game_state().cycle_num && oldball.get().vel.length()>0.5) {
    Line goal_line (goal_center-Vec(0,100), goal_center+Vec(1,-100));
    Line ball_line (oldball.get().pos, oldball.get().pos+oldball.get().vel);
    try{
      target_pos = intersect (goal_line, ball_line);
      if (target_pos.x<-goal_half_width+robot_half_width+100)
	target_pos.x=-goal_half_width+robot_half_width+100;
      else if  (target_pos.x>goal_half_width-robot_half_width-100)
	target_pos.x=goal_half_width-robot_half_width-100;
    }catch(invalid_argument&){;}  // parallele Geraden, fahre in Tormitte
  }
}

void BGoalieRaisedBall::loseControl (const Time& t) throw () {
  was_active=false;
}

DriveVector BGoalieRaisedBall::getCmd(const Time& t) throw () {
//  goto_pos_skill.init (target_pos, Angle::zero, true);
//  DriveVector dest = goto_pos_skill.getCmd (t);
//  dest.kick=false;

  
  // Variante: stehenbleiben!!!!
  DriveVector dest;
  dest.vtrans.x=dest.vtrans.y=dest.vrot=0;
  dest.kick=false;
  return dest;

/*  
  //immer in die Mitte
  const FieldGeometry& fg (MWM.get_field_geometry());
  goal_center = Vec(0, -0.5*fg.field_length);

  goto_pos_skill.init (goal_center, Angle::zero, true);
  DriveVector dest = goto_pos_skill.getCmd (t);
  dest.kick=false;
  return dest;
  */
}
