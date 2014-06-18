#include "SBoostToPos.h"

#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Predicates/freeCorridor.h"
#include <cmath>

using namespace Tribots;
using namespace std;

#define MYLOUT LOUT << "<" << __PRETTY_FUNCTION__ << "> "

SBoostToPos::SBoostToPos(double _mindist , bool _withBall , double _max_abs_angle_diff, double _lookAhead)
  : Skill("SBoostToPos")
{
  withBall = _withBall;
  max_abs_angle_diff = _max_abs_angle_diff;
  mindist = _mindist;
  lookAhead = _lookAhead;
  target = Vec(0,0);
}

void SBoostToPos::setTargetPos(const Vec& _target) throw(TribotsException)
{
  target = _target;
}

bool SBoostToPos::checkInvocationCondition(const Time& t) throw()
{
  // TODO: Hindernisse checken, eigene Roboter rausschmeissen
  // TODO: evtl. mitzählen und von Batteriespannung abhängig machen

  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  bool res = true;

  if (withBall) {
    Frame2d world2robot		= WBOARD->getAbs2RelFrame(t);
    Vec     target_rel		= world2robot * target;
    double  diffang		= Vec(0,1).angle(target_rel).get_rad_pi(); // -pi,pi

    res &= fabs(diffang) < max_abs_angle_diff;
    LOUT << "SBoostToPos: diffang: " << diffang << " < " << max_abs_angle_diff << "? " << res << "\n"; 
    if(!res) return false;
    res &= WBOARD->doPossessBall(t);
    if(!res) return false;
  }
  const RobotLocation& robot = MWM.get_robot_location(t);
  
  //double disttoline = obstacle_distance_to_line_inside_field ( robot.pos , target, obst );
  double disttoline = obstacle_distance_to_line_inside_field ( robot.pos , robot.pos + (target- robot.pos).normalize() * lookAhead, obstacles );
  
  res &= disttoline > 500.0;
  LOUT << "SBoostToPos: obst dist to line: " << disttoline << " > 500? " << res << "\n"; 
  if(!res) return false;

  double distanceToTarget = (target - robot.pos).length();
  res &= distanceToTarget > mindist;
  LOUT << "SBoostToPos: distance to target: " << distanceToTarget << " > " << mindist << "? " << res << "\n"; 

  return res;
}

DriveVector SBoostToPos::getCmd(const Time& t) throw(TribotsException)
{
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  double max_acc_per_cycle = 25.0;
  double boost_wheel_vel   = 130.0;

  double w[3];  
  w[0] =  boost_wheel_vel; // max vel
  w[1] = -boost_wheel_vel; // max vel
  w[2] =   0.0;

  Time trd;
  RobotData rd = MWM.get_robot_data(trd);

  unsigned int rv_id = 0;
  unsigned int lv_id = 1;
  unsigned int back_id = 2;
  
  Frame2d  world2robot = WBOARD->getAbs2RelFrame(t);
  Vec      target_rel  = world2robot * target;
  Vec      pref_dir    = Vec(0,1);

  if (withBall) {
    pref_dir = Vec(0,1);
    rv_id   = 0;
    lv_id   = 1;
    back_id = 2;
  } else {
    int dir_id=0;
    double target_ang = Vec(0,1).angle(target_rel).get_rad_pi(); // -pi,pi
    if (target_ang >= -M_PI/3.0 &&  target_ang <= M_PI/3.0) dir_id = 0;
    else if (target_ang >= M_PI/3.0 &&  target_ang <= M_PI) dir_id = 1;
    else dir_id = 2;
    
    switch (dir_id) {
    case 0:
      pref_dir = Vec(0,1);
      rv_id   = 0; lv_id   = 1; back_id = 2;
      break;
    case 1:
      pref_dir = Vec(0,1).rotate(Angle(2*M_PI / 3.0));
      rv_id   = 1; lv_id   = 2; back_id = 0;
      break;
    case 2:
      pref_dir = Vec(0,1).rotate(-Angle(2*M_PI / 3.0));;
      rv_id   = 2; lv_id   = 0; back_id = 1;
      break;
    default:
      MYLOUT << "dir_id falsch ... sollte nicht passieren";
    }
  }
 
  w[rv_id]   =  boost_wheel_vel;
  w[lv_id]   = -boost_wheel_vel;
  w[back_id] =    0.0;

  // Simulator hat keine vernünftige Information in RobotData
#ifndef SIMULATOR    
  // Beschleunigungskontrolle
  if ( rd.wheel_vel[rv_id]> boost_wheel_vel ) w[rv_id] = boost_wheel_vel;
  else 
    {  
      double e_rv = 0.0;
      e_rv = boost_wheel_vel - rd.wheel_vel[rv_id];
      if (e_rv > max_acc_per_cycle) e_rv = max_acc_per_cycle;
      w[rv_id] = rd.wheel_vel[rv_id] + e_rv;
    }
  
  if ( rd.wheel_vel[lv_id]< -boost_wheel_vel ) w[lv_id] = -boost_wheel_vel;
  else 
    {  
      double e_lv = 0.0;
      e_lv = -boost_wheel_vel - rd.wheel_vel[lv_id];
      if (e_lv < -max_acc_per_cycle) e_lv = -max_acc_per_cycle;
      w[lv_id] = rd.wheel_vel[lv_id] + e_lv;
    }

  if (fabs(w[rv_id]) < fabs(w[lv_id])) w[lv_id] = -w[rv_id];
  if (fabs(w[lv_id]) < fabs(w[rv_id])) w[rv_id] = -w[lv_id];
#else
  w[rv_id]   =  70;
  w[lv_id]   = -70;
  w[back_id] = 0.0;
#endif
  

  // Ausrichtungskorrektur
  double   diffang     = pref_dir.angle(target_rel).get_rad_pi(); // -pi,pi
  if (fabs(diffang) > 0.1) { w[back_id] = diffang * 30.0 + (diffang/fabs(diffang))*10.; }
   
  const RobotLocation& robot = MWM.get_robot_location(t);
  // setzen
  LOUT << "% solid grey " << LineSegment(robot.pos, robot.pos + (pref_dir * robot.heading).normalize()*2000.) << endl;
  LOUT << "\% grey " << Circle(target, 150.) << endl;
  
  LOUT << "\% grey " << Quadrangle (robot.pos, robot.pos + (target- robot.pos).normalize() * lookAhead, 1000.0) << endl;

  LOUT << "SBoostToPos: " << w[0] << " " << w[1] << " " << w[2] << "\n";
  DriveVector dv(w[0] , w[1] , w[2] , false , WHEELVELOCITY);  

  cycles_active++;

  return dv;
}
    
void SBoostToPos::gainControl(const Time&) throw(TribotsException)
{
  cycles_active = 0;
}

void SBoostToPos::loseControl(const Time&) throw(TribotsException)
{
  ;
}


