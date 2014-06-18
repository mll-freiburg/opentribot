
#include "BGameStopped.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>
#include <iostream>

using namespace std;
using namespace Tribots;

BGameStopped::BGameStopped () throw () 
  : Behavior ("BGameStopped"), stopped(false), cycle(0), initial_speed(0.) {;}

bool BGameStopped::checkInvocationCondition(const Time&) throw() {
  return MWM.get_game_state().refstate == stopRobot;
}

bool BGameStopped::checkCommitmentCondition(const Time&) throw() {
  return MWM.get_game_state().refstate == stopRobot;
}

void BGameStopped::gainControl(const Time& t) throw() {
  stopped = false; cycle=0;
  initial_speed = MWM.get_robot_location(t).vtrans.length();
  LOUT << "BGameStopped gained control." << endl;
}

DriveVector BGameStopped::getCmd(const Time& t) throw() {
  DriveVector dest;
  dest.vrot=0; dest.kick=false;
  if (stopped) {
    dest.vtrans.x=dest.vtrans.y=0;
  }
  else {
    const RobotLocation& robot = MWM.get_robot_location(t);
    if (robot.vtrans.length() < 0.2 || cycle++ > 15) {
      stopped = true;
      LOUT << "Stopped is true." << endl;
    }
    dest.vtrans = pow(0.92,cycle) * initial_speed * (robot.vtrans.normalize()/robot.heading);
    LOUT << "Initial speed: " << initial_speed << " cylce: " << cycle 
         << " vtrans: " << dest.vtrans << endl;
  }
  dest.kick=false;
  return dest;
}

