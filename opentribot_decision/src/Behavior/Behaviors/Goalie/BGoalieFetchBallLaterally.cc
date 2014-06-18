
#include "BGoalieFetchBallLaterally.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieFetchBallLaterally::BGoalieFetchBallLaterally (double faw, Angle ma, bool hw) throw () : Behavior ("BGoalieFetchBallLaterally") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  const RobotProperties& rp (MWM.get_robot_properties());
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;
  goal_post_right.x=0.5*fg.goal_width;
  goal_post_right.y=-0.5*fg.field_length;
  goal_post_left.x=-0.5*fg.goal_width;
  goal_post_left.y=-0.5*fg.field_length;
  Vec fa1 (0.5*fg.goal_width, -0.5*fg.field_length+600);
  Vec fa2 (0.5*fg.goal_width+faw,  -0.5*fg.field_length+600+faw);
  Vec fa3 (0.5*fg.goal_width+faw, -0.5*fg.field_length);
  fetching_area_right = Quadrangle (fa1, fa2, fa3, goal_post_right);
  max_angle = ma;
  has_goalie_wings = hw;
  ball_radius = 0.5*fg.ball_diameter;
  min_robot_radius = rp.min_robot_radius;
}

bool BGoalieFetchBallLaterally::checkInvocationCondition (const Time& texec) throw () {
  BallLocation ball = MWM.get_ball_location (texec);
  return checkCommitmentCondition (texec) && (ball.velocity.length()<0.6);
}

bool BGoalieFetchBallLaterally::checkCommitmentCondition (const Time& texec) throw () {
  BallLocation ball = MWM.get_ball_location (texec);
  RobotLocation robot = MWM.get_robot_location (texec);
  bool ball_known = (ball.pos_known==BallLocation::known);
  bool ball_velocity_small = (ball.velocity.length()<1);
  bool robot_in_working_area = ((pa1.x<=robot.pos.x) && (pa1.y<=robot.pos.y) && (robot.pos.x<=pa2.x) && (robot.pos.y<=pa2.y))
  		|| ((goal_post_left.x <= robot.pos.x) && (robot.pos.x <= goal_post_right.x) && (robot.pos.y <= goal_post_left.y));
  Vec br (abs(ball.pos.x), ball.pos.y);
  bool ball_in_fetching_area = fetching_area_right.is_inside (br);
  return ball_known && ball_velocity_small && robot_in_working_area && ball_in_fetching_area;
}

DriveVector BGoalieFetchBallLaterally::getCmd(const Time& texec) throw () {
  DriveVector dest;
  BallLocation ball = MWM.get_ball_location (texec);  
  RobotLocation robot = MWM.get_robot_location (texec);  
  int dir = (ball.pos.x>0 ? +1 : -1);
  if (dir*robot.pos.x>dir*ball.pos.x || robot.pos.y>ball.pos.y+500) {
    // zunaechst um den Ball herumfahren
    goto_ball_skill.init (-dir*max_angle, dir);
    dest = goto_ball_skill.getCmd (texec);
  } else if (has_goalie_wings && dir*(ball.pos.x-robot.pos.x)<min_robot_radius+ball_radius+200 && abs(ball.pos.y-robot.pos.y)<200) {
    // drehen, um den Ball mit den Fluegeln wegzuspitzeln
    goto_pos_skill.init (ball.pos.toVec(), Angle::zero, false);
    dest = goto_pos_skill.getCmd (texec);
  } else {
    // Richtung Ball fahren
    goto_pos_skill.init (ball.pos.toVec(), -dir*max_angle, false);
    dest = goto_pos_skill.getCmd (texec);
  }
  dest.kick=false;
  return dest;
}

