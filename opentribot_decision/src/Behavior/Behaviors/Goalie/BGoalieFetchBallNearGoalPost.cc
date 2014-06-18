
#include "BGoalieFetchBallNearGoalPost.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieFetchBallNearGoalPost::BGoalieFetchBallNearGoalPost () throw () : Behavior ("BGoalieFetchBallNearGoalPost") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  const RobotProperties& rp (MWM.get_robot_properties());
  robot_half_width = 0.5*rp.robot_width;
  fa1=Vec (0.5*fg.goal_width-robot_half_width-100, -0.5*fg.field_length-500);
  fa2=Vec (0.5*fg.goal_width+100, -0.5*fg.field_length+500);
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;
  goal_post_right.x=0.5*fg.goal_width;
  goal_post_right.y=-0.5*fg.field_length;
  goal_post_left.x=-0.5*fg.goal_width;
  goal_post_left.y=-0.5*fg.field_length;
}

bool BGoalieFetchBallNearGoalPost::checkInvocationCondition (const Time& texec) throw () {
  BallLocation ball = MWM.get_ball_location (texec);
  RobotLocation robot = MWM.get_robot_location (texec);
  bool ball_known = (ball.pos_known==BallLocation::known);
  bool ball_velocity_really_small = (ball.velocity.length()<0.6);
  bool robot_in_working_area = ((pa1.x<=robot.pos.x) && (pa1.y<=robot.pos.y) && (robot.pos.x<=pa2.x) && (robot.pos.y<=pa2.y))
  		|| ((goal_post_left.x <= robot.pos.x) && (robot.pos.x <= goal_post_right.x) && (robot.pos.y <= goal_post_left.y));
  bool ball_in_fetching_area = (fa1.x<=abs(ball.pos.x)) && (fa1.y<=ball.pos.y) && (abs(ball.pos.x)<=fa2.x) && (ball.pos.y<=fa2.y);
  return ball_known && ball_velocity_really_small && robot_in_working_area && ball_in_fetching_area;
}

bool BGoalieFetchBallNearGoalPost::checkCommitmentCondition (const Time& texec) throw () {
  BallLocation ball = MWM.get_ball_location (texec);
  RobotLocation robot = MWM.get_robot_location (texec);
  bool ball_known = (ball.pos_known==BallLocation::known);
  bool robot_in_working_area = ((pa1.x<=robot.pos.x) && (pa1.y<=robot.pos.y) && (robot.pos.x<=pa2.x) && (robot.pos.y<=pa2.y))
  		|| ((goal_post_left.x <= robot.pos.x) && (robot.pos.x <= goal_post_right.x) && (robot.pos.y <= goal_post_left.y));
  bool ball_in_fetching_area = (fa1.x<=abs(ball.pos.x)) && (fa1.y<=ball.pos.y) && (abs(ball.pos.x)<=fa2.x) && (ball.pos.y<=fa2.y);
  bool robot_in_goal = (goal_post_left.x <= robot.pos.x && robot.pos.x <= goal_post_right.x && robot.pos.y <= goal_post_right.y+400);
  return ball_known && robot_in_working_area && (ball_in_fetching_area || robot_in_goal);
}

DriveVector BGoalieFetchBallNearGoalPost::getCmd(const Time& texec) throw () {
  DriveVector dest;
  BallLocation ball = MWM.get_ball_location (texec);  
  int dir = (ball.pos.x>0 ? +1 : -1);
  Vec attack_pos (dir*(fa2.x-robot_half_width-100), ball.pos.y);

  goto_ball_skill.init (-dir*Angle::twelvth, dir);
  
  dest = goto_ball_skill.getCmd (texec, attack_pos);
  dest.kick=false;
  return dest;
}

