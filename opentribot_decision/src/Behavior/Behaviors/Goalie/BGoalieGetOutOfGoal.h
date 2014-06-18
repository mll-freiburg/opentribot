
#ifndef Tribots_BGoalieGetOutOfGoal_h
#define Tribots_BGoalieGetOutOfGoal_h

#include "../../Behavior.h"

namespace Tribots {

  /** Faehrt den Roboter aus dem eigenen Tor heraus oder von einer Position neben dem Tor vor das Tor zurueck */
  class BGoalieGetOutOfGoal : public Behavior {
  public:
    /** KOnstruktor, uebergeben wird Mindestabstand vom Tor in mm */
    BGoalieGetOutOfGoal (double) throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw ();
    /** Invocation Bedingung: Roboter befindet sich hinter der Torauslinie */
    bool checkInvocationCondition (const Time&) throw ();
    /** Commitment Condition: wie Invocation Bedingung */
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Vec point_right;
    double robot_radius;
  };

}

#endif 
