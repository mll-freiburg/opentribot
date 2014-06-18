
#include "BLeaveGoal.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>

using namespace Tribots;
using namespace std;

BLeaveGoal::BLeaveGoal () : Behavior ("BLeaveGoal") {
  goto_skill.set_dynamics (1.0, 1.0);
}

bool BLeaveGoal::checkInvocationCondition(const Time& t) throw() {
  const FieldGeometry& fg (MWM.get_field_geometry());
  Vec robotpos = MWM.get_robot_location (t).pos;
  return (abs(robotpos.x)<0.5*fg.goal_width && abs(robotpos.y)<0.5*fg.field_length+fg.goal_length && abs(robotpos.y)>0.5*fg.field_length-MWM.get_robot_properties().max_robot_radius);
}

bool BLeaveGoal::checkCommitmentCondition(const Time& t) throw() {
  const FieldGeometry& fg (MWM.get_field_geometry());
  Vec robotpos = MWM.get_robot_location (t).pos;
  double sec = 2*MWM.get_robot_properties().max_robot_radius;
  return (abs(robotpos.x)<0.5*fg.goal_width+100 && abs(robotpos.y)<0.5*fg.field_length+fg.goal_length+sec && abs(robotpos.y)>0.5*fg.field_length-1.5*sec);  
}

DriveVector BLeaveGoal::getCmd(const Time& t) throw(TribotsException) {
  const FieldGeometry& fg (MWM.get_field_geometry());  
  double y = 0.5*fg.field_length+fg.goal_length;
  double x = 0.5*fg.goal_width;
  std::vector<Line> barriers (4);
  barriers[0] = Line (Vec(x,y),Vec(-x,y));
  barriers[1] = Line (Vec(x,-y),Vec(-x,-y));
  barriers[2] = Line (Vec(x,y),Vec(x,-y));
  barriers[3] = Line (Vec(-x,y),Vec(-x,-y));
  
  goto_skill.init (Vec(0,0), MWM.get_robot_location(t).heading, true);
//  goto_skill.init_barrier (barriers);
  return goto_skill.getCmd(t);
}
