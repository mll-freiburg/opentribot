
#include "BGoalieGetAwayFromGoalPosts.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

namespace {

  // diese Dinge sind hier plaziert wegen const-Klausel in Bedingungsabfragen (schlechte Designentscheidung):
  unsigned long int iteration;  // Iteration, zu der letzmalig folgender Wert aus dem Weltmodell geholt wurden
  RobotLocation robot_exec;  // Roboterposition zum Ausfuehrungszeitpunkt
  RobotLocation robot_vis;

}


BGoalieGetAwayFromGoalPosts::BGoalieGetAwayFromGoalPosts () throw () : Behavior ("BGoalieGetAwayFromGoalPosts") {
  iteration=0;
  const FieldGeometry& fg (MWM.get_field_geometry());
  goal_post_right.x = 0.5*fg.goal_width;
  goal_post_right.y = -0.5*fg.field_length;
  robot_radius = MWM.get_robot_properties().max_robot_radius;
}

bool BGoalieGetAwayFromGoalPosts::checkInvocationCondition (const Time & texec) throw () {
  if (iteration!=MWM.get_game_state().cycle_num) {
    robot_exec = MWM.get_robot_location (texec);
    iteration = MWM.get_game_state().cycle_num;
    Time tvis;
    MWM.get_slfilter_robot_location (tvis); // um tvis zu bekommen
    robot_vis = MWM.get_robot_location (tvis);
  }

  return (robot_vis.pos.y<goal_post_right.y+robot_radius && abs(abs(robot_vis.pos.x)-goal_post_right.x)<robot_radius && abs(robot_vis.pos.x)>500 && (robot_vis.heading.in_between (Angle::sixth, Angle::five_sixth)));
}

bool BGoalieGetAwayFromGoalPosts::checkCommitmentCondition (const Time & texec) throw () {
  return checkInvocationCondition (texec);
}

DriveVector BGoalieGetAwayFromGoalPosts::getCmd(const Time& texec) throw() {
  DriveVector dest;
  checkInvocationCondition (texec);  // um ggf. robot_exec zu holen
  RobotLocation robot_slfilter;  // SL-gefilterte Roboterposition
  Time tvis;
  robot_slfilter = MWM.get_slfilter_robot_location (tvis);
  dest.vtrans.x=0;
  dest.vtrans.y=0.5;
  dest.vrot=0;
  dest.kick=false;
  if (abs(robot_slfilter.pos.x)<goal_post_right.x) {
    // 1. Fall: Roboter innerhalb des Tores
    if (robot_vis.pos.x+robot_radius+10 > goal_post_right.x)
      dest.vtrans.x=-0.5;  // in Weltkoordinaten nach links fahren, da zu nahe an rechter Toraussenwand
    else if (robot_vis.pos.x-robot_radius-10 < -goal_post_right.x)
      dest.vtrans.x=0.5;  // in Weltkoordinaten nach rechts fahren, da zu nahe an linker Toraussenwand
    else
      dest.vtrans.x=0;
  } else {
    // 2. Fall Roboter ausserhalb des Tores
    if (robot_vis.pos.x+robot_radius+50 > -goal_post_right.x)
      dest.vtrans.x=-0.5;  // in Weltkoordinaten nach links fahren, da zu nahe an linker Toraussenwand
    else if (robot_vis.pos.x-robot_radius-50 < goal_post_right.x)
      dest.vtrans.x=0.5;  // in Weltkoordinaten nach rechts fahren, da zu nahe an rechter Toraussenwand
    else
      dest.vtrans.x=0;
  }

  // von Welt- in Roboterkoordinaten umrechnen:
  dest.vtrans/=robot_vis.heading;
  return dest;
}
