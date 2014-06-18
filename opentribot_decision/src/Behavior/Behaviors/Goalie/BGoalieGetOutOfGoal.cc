
#include "BGoalieGetOutOfGoal.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BGoalieGetOutOfGoal::BGoalieGetOutOfGoal (double my) throw () : Behavior ("BGoalieGetOutOfGoal") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  point_right.x = 0.5*fg.goal_width;
  point_right.y = -0.5*fg.field_length+my;
  robot_radius = MWM.get_robot_properties().max_robot_radius;
}

bool BGoalieGetOutOfGoal::checkInvocationCondition (const Time & texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  return (robot_exec.pos.y<point_right.y);
}

bool BGoalieGetOutOfGoal::checkCommitmentCondition (const Time & texec) throw () {
  return checkInvocationCondition (texec);
}

DriveVector BGoalieGetOutOfGoal::getCmd(const Time& texec) throw() {
  DriveVector dest;
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  RobotLocation robot_slfilter;  // SL-gefilterte Roboterposition
  Time tvis;
  robot_slfilter = MWM.get_slfilter_robot_location (tvis);
  dest.vtrans.x=0;
  dest.vtrans.y=0.5;
  dest.vrot=0;
  dest.kick=false;
  if (abs(robot_slfilter.pos.x)<point_right.x) {
    // 1. Fall: Roboter innerhalb des Tores
    if (robot_exec.pos.x+robot_radius+10 > point_right.x)
      dest.vtrans.x=-0.5;  // in Weltkoordinaten nach links fahren, da zu nahe an rechter Toraussenwand
    else if (robot_exec.pos.x-robot_radius-10 < -point_right.x)
      dest.vtrans.x=0.5;  // in Weltkoordinaten nach rechts fahren, da zu nahe an linker Toraussenwand
    else
      dest.vtrans.x=0;
  } else {
    // 2. Fall Roboter ausserhalb des Tores
    if (robot_exec.pos.x+robot_radius+50 > -point_right.x)
      dest.vtrans.x=-0.5;  // in Weltkoordinaten nach links fahren, da zu nahe an linker Toraussenwand
    else if (robot_exec.pos.x-robot_radius-50 < point_right.x)
      dest.vtrans.x=0.5;  // in Weltkoordinaten nach rechts fahren, da zu nahe an rechter Toraussenwand
    else
      dest.vtrans.x=0;
  }

  // von Welt- in Roboterkoordinaten umrechnen:
  dest.vtrans/=robot_exec.heading;
  return dest;
}
