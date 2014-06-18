
#include "BGoalieAttackBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/stringconvert.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieAttackBall::BGoalieAttackBall (bool co, Vec a1, Vec a2, bool kp, SPhysGotoPos* sp) throw () : Behavior ("BGoalieAttackBall"), consider_obstacles (co), kick_permission (kp) {
  const FieldGeometry& fg (MWM.get_field_geometry());
  const RobotProperties& rp (MWM.get_robot_properties());
  goal_post_right = Vec (0.5*fg.goal_width, -0.5*fg.field_length);
  goal_post_left = Vec (-0.5*fg.goal_width, -0.5*fg.field_length);
  aa1 = a1;
  aa2 = a2;
  robot_half_width = 0.5*rp.robot_width;
  kick_distance = 0.5*fg.ball_diameter+rp.kicker_distance+100;
  kicker_half_width = 0.5*rp.kicker_width;
  ball_radius = 0.5*fg.ball_diameter;
  if (sp)
    goto_pos_skill=sp;
  else
    goto_pos_skill=&own_goto_pos_skill;
}

void BGoalieAttackBall::gainControl (const Time&) throw(TribotsException) {
  goto_pos_skill->set_dynamics (2.0, 2.0, 5.0, 8.0);
}

bool BGoalieAttackBall::checkInvocationCondition (const Time& texec) throw () {
  const RobotLocation& robot (MWM.get_robot_location (texec));
  const BallLocation& ball (MWM.get_ball_location (texec));

  bool ball_known = (ball.pos_known==BallLocation::known);
  bool robot_in_working_area = (aa1.x<=robot.pos.x && aa1.y<=robot.pos.y && robot.pos.x<=aa2.x && robot.pos.y<=aa2.y);
  robot_in_working_area |= (goal_post_left.x <= robot.pos.x && robot.pos.x <= goal_post_right.x && robot.pos.y <= goal_post_left.y);
  bool ball_in_attack_area = (aa1.x<=ball.pos.x && aa1.y<=ball.pos.y && ball.pos.x<=aa2.x && ball.pos.y<=aa2.y);
  ball_in_attack_area &= ((ball.pos.x-goal_post_left.x)>=-2*(ball.pos.y-goal_post_left.y)) && ((ball.pos.x-goal_post_right.x)<=2*(ball.pos.y-goal_post_right.y));
  bool robot_headed_to_ball = (abs(((ball.pos-robot.pos).angle()-Angle::quarter-robot.heading).get_deg_180())<20);
  bool robot_behind_ball = (robot.pos.y+100<ball.pos.y);
  bool high_ball = (ball.pos.z>300 || ball.velocity.z>0.5);
  bool ball_slow = (ball.velocity.toVec().length()<0.8);
  bool ball_close = (ball.pos.toVec()-robot.pos).length()<1000;
  bool ball_movement_similar = (robot.pos-ball.pos.toVec()).angle(-ball.velocity.toVec()).in_between (-Angle::deg_angle(5), Angle::deg_angle(5));
  return ball_known
      && robot_in_working_area
      && robot_headed_to_ball
      && robot_behind_ball
      && ball_in_attack_area
      && !high_ball
      && ( ball_slow || ball_close || ball_movement_similar);
}

bool BGoalieAttackBall::checkCommitmentCondition (const Time& texec) throw () {
  return checkInvocationCondition (texec);
}

DriveVector BGoalieAttackBall::getCmd(const Time& texec) throw () {
  DriveVector dest;
  const RobotLocation& robot (MWM.get_robot_location (texec));
  const BallLocation& ball (MWM.get_ball_location (texec));
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(texec);
    
  Vec tgpos = ball.pos.toVec();
  /* auskommentiert, da es nicht viel bringt; z-Geschwindigkeitsschaetzung zu ungenau
  try{
    // Unterlaufen des Balls verhindern
    Vec basepos = intersect (Line(goal_post_left+Vec(0,300), goal_post_right+Vec(0,300)), Line(ball.pos.toVec(), ball.pos.toVec()+ball.velocity.toVec()));
    Vec tgpos0 = tgpos;
    double ball_z0 = ball.pos.z;
    double ball_vz0 = ball.velocity.z;
    for (unsigned int i=0; i<=5; i++) {
      tgpos = tgpos0+static_cast<double>(i)/5.0*(basepos-tgpos0);
      double tt=(tgpos-basepos).length()/ball.velocity.toVec().length();
      double ball_z = ball_z0-0.5e-3*6.0*(tt*tt);
      double ball_vz = ball_vz0-6.0e-3*tt;
      LOUT << "BGAB: " << tgpos << ' ' << ball_z << ' ' << ball_vz << '\n';
      if (ball_z<600 && (ball_vz<=0 || ball_z+0.5e3*(ball_vz*ball_vz)/6.0<600))
        break;
      LOUT << "BGAB: Weg verkuerzt, um Unterlaufen des Balls zu verhindern\n";
    }
  }catch(std::invalid_argument&){ ; }  // irgendeine geometrische Seltsamkeit
  */

  goto_pos_skill->init (tgpos, robot.heading, false, false, false);
  dest = goto_pos_skill->getCmd (texec);
  dest.kick=false;

  if (kick_permission) {
    Time tkick=texec;
    tkick.add_msec(-200);
    RobotLocation rkick = MWM.get_robot_location (tkick);
    BallLocation bkick = MWM.get_ball_location (tkick);
    try{
      Line robot_mid (rkick.pos, rkick.pos+Vec::unit_vector_y.rotate (rkick.heading));
      Vec pp = robot_mid.perpendicular_point (bkick.pos.toVec());
      dest.kick = (pp-bkick.pos.toVec()).squared_length()<kicker_half_width*kicker_half_width
        && (pp-rkick.pos).angle().in_between (rkick.heading, rkick.heading+Angle::half)
        && (pp-rkick.pos).length()<kick_distance;
    }catch(std::invalid_argument&){;} // Ball- und Roboterposition fallen aufeinander
  }

  if (consider_obstacles) {
    Line kick_line (robot.pos, robot.pos+Vec::unit_vector_y.rotate(robot.heading));
    double len2 = (robot.pos-ball.pos.toVec()).squared_length();
    
    vector<ObstacleDescriptor>::const_iterator it = obstacles.begin();
    vector<ObstacleDescriptor>::const_iterator itend = obstacles.end();
    while (it<itend) {
      Vec pp = kick_line.perpendicular_point (it->pos);
      if ((pp-robot.pos).squared_length()<=len2 && (pp-ball.pos.toVec()).squared_length()<=len2 && (it->pos-pp).length()<robot_half_width+0.5*it->width-300) {
        dest.kick=false;
        dest.vrot=0;
        dest.vtrans.x=dest.vtrans.y=0;
        LOUT << "Greife Ball nicht an wegen Hindernissen auf der Verbindungsstrecke\n";
        break;
      }
      if (it->pos.y>robot.pos.y && (it->pos-robot.pos).length()<1500 && (it->pos-pp).length()-0.5*it->width<ball_radius) {
        dest.kick = false;
        LOUT << "schiesse nicht wegen Hindernissen in Schussbahn\n";
        break;
      }
      it++;
    }
  }
  return dest;
}

void BGoalieAttackBall::updateTactics (const TacticsBoard& tb) throw () {
  double attar=-1;
  bool success = string2double (attar, tb[string("GoalieAttackArea")]);
  if (success && attar>=0 && attar<=5000) {
    const FieldGeometry& fg (MWM.get_field_geometry());
    aa1.x=-attar;
    aa2.x=attar;
    aa1.y=-0.5*fg.field_length;
    aa2.y=-0.5*fg.field_length+attar;
  }
}
