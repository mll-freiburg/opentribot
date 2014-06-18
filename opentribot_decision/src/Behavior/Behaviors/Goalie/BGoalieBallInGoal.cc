
#include "BGoalieBallInGoal.h"
#include "../../../WorldModel/WorldModel.h"

using namespace Tribots;

BGoalieBallInGoal::BGoalieBallInGoal () throw () : Behavior ("BGoalieBallInGoal") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  goal_post_left.x = -0.5*fg.goal_width;
  goal_post_right.x = 0.5*fg.goal_width;
  pa.x = 0.5*fg.field_width;
  pa.y = -0.5*fg.field_length;
  goal_post_left.y = goal_post_right.y = -0.5*fg.field_length;
}

bool BGoalieBallInGoal::checkInvocationCondition (const Time& texec) throw () {
  const BallLocation& ball = MWM.get_ball_location(texec);
  return (ball.pos_known==BallLocation::known && ball.pos.y+150<goal_post_left.y && ball.pos.x>goal_post_left.x && ball.pos.x<goal_post_right.x);
}

bool BGoalieBallInGoal::checkCommitmentCondition (const Time& texec) throw () {
  return checkInvocationCondition(texec);
}

DriveVector BGoalieBallInGoal::getCmd(const Time& time1) throw () {
  DriveVector dv;
  const RobotLocation& robot = MWM.get_robot_location (time1);

  if ((robot.pos.x< -1000+goal_post_left.x )||(robot.pos.x> 1000+goal_post_right.x)||(robot.pos.y> 2000+goal_post_left.y)){
    goto_obs_skill.init (Vec(0,pa.y), Angle::zero,true);
    return goto_obs_skill.getCmd(time1);
  }
  else
  {
    dv.vtrans=Vec::zero_vector;
    dv.vrot=0;
  }
  dv.kick=false;
  return dv;
}

