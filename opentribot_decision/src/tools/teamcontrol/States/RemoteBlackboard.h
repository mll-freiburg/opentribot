
#ifndef TribotsTools_RemoteBlackboard_h
#define TribotsTools_RemoteBlackboard_h

#include "CoachState.h"
#include "RemoteRobotState.h"
#include "JoystickState.h"
#include "TeamState.h"
#include "HelpState.h"
#include <vector>

namespace TribotsTools {

  /** Struktur, die alle Teilstrukturen des RemoteBlackboards zusammenfast */
  struct BlackboardState {
    std::vector<RemoteRobotState> robot_state;
    CoachState coach_state;
    JoystickState joystick_state;
    TeamState team_state;
    HelpState help_state;
    /** Pfeil zwischen Zwei Robotern ins Spielfeld einzeichnen:
        Arg1: Quelle (Roboternummer)
        Arg2: Ziel (Roboternummer)
        Arg3: Farbe (red, green, etc. oder color R G B)
        Arg4: Pfeiltext
        Arg5: Wie lange soll der Pfeil eingezeichnet werden (in ms)? */
    void drawRobotArrow (unsigned int, unsigned int, const char*, const char*, unsigned int = 1000);
    /** Pfeil zwischen Zwei Robotern ins Spielfeld einzeichnen:
        Arg1: Roboternummer
        Arg2: Farbe (red, green, etc. oder color R G B)
        Arg3: Text
        Arg4: Wie lange soll der Text eingezeichnet werden (in ms)? */
    void drawRobotText (unsigned int, const char*, const char*, unsigned int = 1000);
    
    RemoteRobotState* get_robot_with_id(unsigned int id);
    const RemoteRobotState* get_robot_with_id(unsigned int id) const;
  };    

  /** Blackboard, um die Daten, die zwischen verschiedenen Teilen ausgetauscht werden mussen, zentral zu halten 
      Als Singleton implementiert */
  class RemoteBlackboard {
  private:
    static RemoteBlackboard* the_blackboard;
    RemoteBlackboard ();
  public:
    static RemoteBlackboard& get_blackboard ();

    BlackboardState state;
  };
  
}

#define REMBB TribotsTools::RemoteBlackboard::get_blackboard().state


#endif
