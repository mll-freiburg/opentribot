
#include "BTestOwnsBallCheck.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Predicates/freeCorridor.h"
#include <cmath>
#include <deque>
#include <algorithm>

using namespace Tribots;
using namespace std;

bool BTestOwnsBallCheck::ballInCheckHalf (Time t) {
  return abs(MWM.get_ball_location (t).pos.x)+1000<0.5*MWM.get_field_geometry().field_width &&
      checkHalf*MWM.get_ball_location (t).pos.y>1000 && 
      abs(MWM.get_ball_location (t).pos.y)+1000<0.5*MWM.get_field_geometry().field_length;
}

bool BTestOwnsBallCheck::ballClose (Time t) {
  Vec ballRelative = (MWM.get_ball_location(t).pos.toVec()-MWM.get_robot_location(t).pos)/MWM.get_robot_location(t).heading;
  Vec velRelative = MWM.get_robot_location(t).vtrans/MWM.get_robot_location(t).heading;
  return (ballRelative.y<400 && ballRelative.y>200 && abs(ballRelative.x)<100 && velRelative.length()>1.2 && (velRelative.angle()-Angle::quarter).in_between (-Angle::twelvth, Angle::twelvth));
}

bool BTestOwnsBallCheck::wayFree (Time t) {
  Vec ball = MWM.get_ball_location(t).pos.toVec();
  Vec robot = MWM.get_robot_location(t).pos;
  double odist1 = obstacle_distance_to_line (ball, ball+1000*(ball-robot).normalize(), MWM.get_obstacle_location(t));
  double odist2 = obstacle_distance_to_line (robot, ball, MWM.get_obstacle_location(t));
  return (odist1>300) && (odist2>300);
}

BTestOwnsBallCheck::BTestOwnsBallCheck (RefereeState ts) : Behavior ("BTestOwnsBallCheck"), testState(ts) {
  gotoPos.set_dynamics (1.5);
  checkHalf=1; 
  wasTestState=false;
}

BTestOwnsBallCheck::~BTestOwnsBallCheck () throw () {;}

void BTestOwnsBallCheck::cycleCallBack(const Time& t) throw () {
  if (MWM.get_game_state().refstate==testState) {
    if (!wasTestState)
      checkHalf=(MWM.get_robot_location(t).pos.y>0 ? +1 : -1);
    wasTestState=true;
  } else {
    wasTestState=false;
  }
}

bool BTestOwnsBallCheck::checkInvocationCondition(const Time& t) throw() {
  return ballInCheckHalf(t) && MWM.get_game_state().refstate==testState && wayFree (t);
}

bool BTestOwnsBallCheck::checkCommitmentCondition(const Time& t) throw() { 
  return checkInvocationCondition(t); 
}

DriveVector BTestOwnsBallCheck::getCmd(const Time& t) throw() {
  bool doCollect=false;
  bool isClose=false;
  bool stopped=(startTime.elapsed_msec()>2000);

  if (stopped) {
    DriveVector dvnull (Vec(0,0),0,false);
    if (startTime.elapsed_msec()>3000)
      startTime.update();
    return dvnull;
  }
  if (ballClose (t)) {
    isClose=true;
    if (startTime.elapsed_msec()>500) {
      doCollect=true;
      distances.push_back (((MWM.get_ball_location(t).pos.toVec()-MWM.get_robot_location(t).pos)/MWM.get_robot_location(t).heading).y);
    }
  } else {
    startTime.update();
  }
  Vec robotBall = MWM.get_ball_location(t).pos.toVec()-MWM.get_robot_location(t).pos;
  gotoPos.init (MWM.get_ball_location(t).pos.toVec(), robotBall.angle()-Angle::quarter, false);
  LOUT << "DistanceToBall: " << robotBall.length() << ' ' << (robotBall.angle()-MWM.get_robot_location(t).heading).get_deg() << ' ' << isClose << ' ' << doCollect << '\n';
  return gotoPos.getCmd (t);
}

void BTestOwnsBallCheck::loseControl (const Time&) throw(TribotsException) {
}

void BTestOwnsBallCheck::gainControl (const Time& t) throw(TribotsException) {
  startTime.update();
}

void BTestOwnsBallCheck::clearCalibration () throw () {
  distances.clear();
}

double BTestOwnsBallCheck::getCalibration () throw () {
  if (distances.size()<20)
    return -1;
  LOUT << "DistanceStatistics: ";
  sort (distances.begin(), distances.end());
  LOUT << distances.size() << "   " << distances[0] 
      << ' ' << distances[distances.size()/8] 
      << ' ' << distances[distances.size()/4] 
      << ' ' << distances[3*distances.size()/8] 
      << ' ' << distances[distances.size()/2] 
      << ' ' << distances[5*distances.size()/8] 
      << ' ' << distances[3*distances.size()/4] 
      << ' ' << distances[7*distances.size()/8] 
      << ' ' << distances[distances.size()-1] << '\n';
  return distances[7*distances.size()/8];
}
