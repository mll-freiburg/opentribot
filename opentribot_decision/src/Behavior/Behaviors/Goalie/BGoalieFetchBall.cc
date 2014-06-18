
#include "BGoalieFetchBall.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieFetchBall::BGoalieFetchBall (double fad) throw () : Behavior ("BGoalieFetchBall") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;
  goal_post_right.x=0.5*fg.goal_width;
  goal_post_right.y=-0.5*fg.field_length;
  goal_post_left.x=-0.5*fg.goal_width;
  goal_post_left.y=-0.5*fg.field_length;
  fa1=goal_post_left-Vec(fad,0);
  fa2=goal_post_right+Vec(fad,1500);
  rp1.x=-250;
  rp2.x=250;
  rp1.y=rp2.y=-0.5*fg.field_length;
  latest_dir=0;
}

bool BGoalieFetchBall::checkInvocationCondition (const Time& texec) throw () {
  const BallLocation& ball (MWM.get_ball_location (texec));
  return checkCommitmentCondition (texec) && (ball.velocity.length()<0.6);
}

bool BGoalieFetchBall::checkCommitmentCondition (const Time& texec) throw () {
  const RobotLocation& robot (MWM.get_robot_location (texec));
  const BallLocation& ball (MWM.get_ball_location (texec));

  bool ball_known = (ball.pos_known==BallLocation::known);
  bool ball_velocity_small = (ball.velocity.length()<1);
  bool robot_in_working_area = ((pa1.x<=robot.pos.x) && (pa1.y<=robot.pos.y) && (robot.pos.x<=pa2.x) && (robot.pos.y<=pa2.y))
  		|| ((goal_post_left.x <= robot.pos.x) && (robot.pos.x <= goal_post_right.x) && (robot.pos.y <= goal_post_left.y));
  bool ball_in_goal = ((goal_post_left.x <= ball.pos.x) && (ball.pos.x <= goal_post_right.x) && (ball.pos.y <= goal_post_left.y));
  bool ball_in_penalty_area = ((fa1.x<=ball.pos.x) && (fa1.y<=ball.pos.y) && (ball.pos.x<=fa2.x) && (ball.pos.y<=fa2.y));
  bool ball_in_cone = ((ball.pos.x-goal_post_left.x)>=-2*(ball.pos.y-goal_post_left.y)) && ((ball.pos.x-goal_post_right.x)<=2*(ball.pos.y-goal_post_right.y));
  bool ball_in_fetching_area = ball_in_goal || (ball_in_penalty_area && ball_in_cone);
  return ball_known && ball_velocity_small && robot_in_working_area && ball_in_fetching_area;
}

void BGoalieFetchBall::gainControl(const Time& t) throw() {
  latest_dir=0;
}

DriveVector BGoalieFetchBall::getCmd(const Time& texec) throw () {
  const RobotLocation& robot (MWM.get_robot_location (texec));
  const BallLocation& ball (MWM.get_ball_location (texec));

  // den Bereich abstecken, in dem der Ball liegt
  if (2*(ball.pos.x-rp1.x)+(ball.pos.y-rp1.y)<=0) 
    latest_dir=-1;  // rechts drehen im linken Bereich des Torraums
  else if (2*(ball.pos.x-rp2.x)-(ball.pos.y-rp2.y)>=0) 
    latest_dir=+1;  // links drehen im rechten Bereich des Torraums
  else if (latest_dir==0) {
    // beide Drehrichtungen moeglich
    if (robot.pos.x>ball.pos.x)
      latest_dir = -1;
    else
      latest_dir = +1;
  }

  goto_ball_skill.init (-latest_dir*Angle::twelvth, latest_dir);
  
  DriveVector dest = goto_ball_skill.getCmd (texec);
  dest.kick=false;
  
  return dest;
}

