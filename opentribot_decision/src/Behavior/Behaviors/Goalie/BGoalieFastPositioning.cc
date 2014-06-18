
#include "BGoalieFastPositioning.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieFastPositioning::BGoalieFastPositioning (Vec home, Vec corner, Angle ma, bool cb) throw () : Behavior ("BGoalieFastPositioning") {
  use_comm_ball = cb;
  max_angle = ma;
  if (corner.x>0) {
    right_end = corner;
    left_end = Vec (-corner.x, corner.y);
  } else {
    left_end = corner;
    right_end = Vec (-corner.x, corner.y);
  }
  Vec center (home.x, ((corner.x-home.x)*(corner.x-home.x)+corner.y*corner.y-home.y*home.y)/(2*(corner.y-home.y)));
  double radius = (right_end-center).length();
  Angle a1 = (right_end-center).angle();
  Angle a2 = (left_end-center).angle();
  positioning_arc = Arc (center, radius, a1, a2);
  const FieldGeometry& fg (MWM.get_field_geometry());
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;
  goal_post_right.x=0.5*fg.goal_width;
  goal_post_right.y=-0.5*fg.field_length;
  goal_post_left.x=-0.5*fg.goal_width;
  goal_post_left.y=-0.5*fg.field_length;
}

bool BGoalieFastPositioning::checkInvocationCondition (const Time& texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  
  return 
    (ball_exec.pos_known == BallLocation::known && 
    pa1.x<=robot_exec.pos.x && pa1.y<=robot_exec.pos.y && 
    robot_exec.pos.x<=pa2.x && robot_exec.pos.y<=pa2.y &&
    ball_exec.pos.y>-0.5*MWM.get_field_geometry().field_length &&
    ball_exec.velocity.toVec().length() >= 2.5 &&
    ball_exec.velocity.toVec().angle().in_between (Angle::five_eighth, Angle::seven_eighth)
    );
}

bool BGoalieFastPositioning::checkCommitmentCondition (const Time& texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  
  return 
    (ball_exec.pos_known == BallLocation::known && 
    pa1.x<=robot_exec.pos.x && pa1.y<=robot_exec.pos.y && 
    robot_exec.pos.x<=pa2.x && robot_exec.pos.y<=pa2.y &&
    ball_exec.pos.y>-0.5*MWM.get_field_geometry().field_length &&
    ball_exec.velocity.toVec().length() >= 2.0 &&
    ball_exec.velocity.toVec().angle().in_between (Angle::five_eighth, Angle::seven_eighth)
    );
}

DriveVector BGoalieFastPositioning::getCmd(const Time& texec) throw () {
  DriveVector dest;
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));

  Line dyna_pos_line (ball_exec.pos.toVec(), ball_exec.pos.toVec()+ball_exec.velocity.toVec());
  Line goal_line (goal_post_left, goal_post_right);

  dest.vtrans = Vec(0.,0.);
  dest.vrot = 0.;
 
  try{
    Vec q = intersect (goal_line, dyna_pos_line);
    bool dangerous_ball = 
      (goal_post_left.x-1000 < q.x && q.x < goal_post_right.x+1000);
    
    if (dangerous_ball && ball_exec.velocity.y < 0.) {
      Line goalie_line (robot_exec.pos, robot_exec.pos + Vec(1000., 0.));

      Vec p = intersect (goalie_line, dyna_pos_line);
      bool left = p.x < robot_exec.pos.x;
      bool near = (abs(p.x-robot_exec.pos.x)<100);
      
      if (!near)
        dest.vtrans = 
          left ? Vec(-2.0, 0.) * -robot_exec.heading :
                 Vec( 2.0, 0.) * -robot_exec.heading;
     
    }
  } catch(invalid_argument&) {
    ; // rollt parallel zur Torauslinie
  }
  dest.kick=false;
  return dest;
}
