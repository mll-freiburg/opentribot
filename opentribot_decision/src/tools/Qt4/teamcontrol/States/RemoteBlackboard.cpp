
#include "RemoteBlackboard.h"


TribotsTools::RemoteBlackboard* TribotsTools::RemoteBlackboard::the_blackboard (NULL);

TribotsTools::RemoteBlackboard::RemoteBlackboard () {;}   // diese Beschraenkung auf max 30 ist willkuerlich, vereinfacht aber die Programmierung

TribotsTools::RemoteBlackboard& TribotsTools::RemoteBlackboard::get_blackboard () {
  if (!the_blackboard)
    the_blackboard = new TribotsTools::RemoteBlackboard;
  return *the_blackboard;
}

void TribotsTools::BlackboardState::drawRobotArrow (unsigned int src, unsigned int target, const char* color, const char* text, unsigned int time) {
  DisplayRobotArrow neu;
  neu.source_id = src;
  neu.target_id = target;
  neu.color = color;
  neu.text = text;
  neu.deadline.add_msec (time);
  help_state.robotarrows.push_back (neu);
}

void TribotsTools::BlackboardState::drawRobotText (unsigned int id, const char* color, const char* text, unsigned int time) {
  DisplayRobotText neu;
  neu.robot_id = id;
  neu.color = color;
  neu.text = text;
  neu.deadline.add_msec (time);
  help_state.robottext.push_back (neu);
}
