
#ifndef Tribots_BGoalieGetAwayFromGoalPosts_h
#define Tribots_BGoalieGetAwayFromGoalPosts_h

#include "../../Behavior.h"

namespace Tribots {

  /** Faehrt den Roboter aus dem eigenen Tor heraus, wenn er verdregt am Torpfosten haengt */
  class BGoalieGetAwayFromGoalPosts : public Behavior {
  public:
    /** Konstruktor */
    BGoalieGetAwayFromGoalPosts () throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw ();
    /** Invocation Bedingung: Roboter befindet sich hinter der Torauslinie */
    bool checkInvocationCondition (const Time&) throw ();
    /** Commitment Condition: wie Invocation Bedingung */
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Vec goal_post_right;
    double robot_radius;
  };

}

#endif 
