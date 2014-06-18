
#include "BTestAllDirections.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Predicates/freeCorridor.h"
#include <cmath>

using namespace Tribots;
using namespace std;


bool BTestAllDirections::wayFree (Time t, Angle a, bool strict) {
  Vec robot = MWM.get_robot_location(t).pos;
  Angle head = MWM.get_robot_location(t).heading;
  Vec target = robot+(strict ? 2000 : 1000)*Vec::unit_vector(a+head);
  double odist2 = obstacle_distance_to_line (robot, target, MWM.get_obstacle_location(t));
  LineSegment way (robot, target);
  double bdist = way.distance(MWM.get_ball_location(t).pos.toVec());
  LOUT << "% " << (half*target.y>0 ? "blue" : "yellow") << " cross " << target <<"\n";
  return (odist2>500) && (bdist>500) &&
      (abs(target.x)<0.5*MWM.get_field_geometry().field_width) &&
      (abs(target.y)<0.5*MWM.get_field_geometry().field_length) &&
      (half*target.y>0);
}

BTestAllDirections::BTestAllDirections (RefereeState ts) : Behavior ("BTestAllDirections"), testState(ts) {
  msecDir[0]=msecDir[1]=msecDir[2]=0;
  angleDir[0]=Angle::deg_angle(90);
  angleDir[1]=Angle::deg_angle(210);
  angleDir[2]=Angle::deg_angle(330);
  latestDir=0;
  msecMax=2000;
  half=-1;
}

BTestAllDirections::~BTestAllDirections () throw () {;}


bool BTestAllDirections::checkInvocationCondition(const Time& t) throw() {
  return MWM.get_game_state().refstate==testState &&
      MWM.get_robot_location(t).vtrans.length()<0.2;
}

bool BTestAllDirections::checkCommitmentCondition(const Time& t) throw() {
  return MWM.get_game_state().refstate==testState &&
      wayFree (t, angleDir[latestDir], false) &&
      msecDir[latestDir]<msecMax;
}

DriveVector BTestAllDirections::getCmd(const Time& t) throw() {
  if (waitCycle==0)
    msecDir[latestDir]+=t.diff_msec(latestTimestamp);
  latestTimestamp=t;

  DriveVector dv (Vec(0,0), 0, false);
  if (waitCycle>0) {
    waitCycle--;
    return dv;
  }
  if (wayFree(t, angleDir[latestDir], false) && (msecDir[latestDir]<msecMax)) {
    // bisherige Richtung weiterfahren
    dv.vtrans=Vec::unit_vector(angleDir[latestDir]);
    return dv;
  }
  waitCycle=5;
  // neue Richtung suchen
  unsigned int nextDir=999;
  unsigned int freeDir=999;
  for (unsigned int i=0; i<3; i++) {
    if (wayFree (t, angleDir[i], true)) {
      freeDir=i;
      if (msecDir[i]<1) {
        nextDir=i;
      }
    }
  }
  if (nextDir<100) {
    latestDir=nextDir;
  } else {
    for (unsigned int i=0; i<3; i++)
      msecDir[i]=0;
    if (freeDir<100)
      latestDir=freeDir;
    else {
      // keine freie Richtung, also stehenbleiben
      return dv;
    }
  }
  return dv;
}

void BTestAllDirections::loseControl (const Time&) throw(TribotsException) {
}

void BTestAllDirections::gainControl (const Time& t) throw(TribotsException) {
  latestTimestamp.update();
  waitCycle=0;
  half = (MWM.get_robot_location(t).pos.y>0 ? +1 : -1);
}

void BTestAllDirections::cycleCallBack(const Time& t) throw () {
  RefereeState state = MWM.get_game_state().refstate;
  if (state==testState && latestState!=testState)
    half = (MWM.get_robot_location(t).pos.y>0 ? +1 : -1);
  latestState=state;
}
