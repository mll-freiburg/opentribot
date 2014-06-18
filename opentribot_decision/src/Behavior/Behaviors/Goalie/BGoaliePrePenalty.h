
#ifndef Tribots_BGoaliePrePenalty_h
#define Tribots_BGoaliePrePenalty_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"

namespace Tribots {

  /** Positioniert den Goalie in der Mitte des Tores bevor bei 
      einem Penalty der Ball gespielt wurde */
  class BGoaliePrePenalty : public Behavior {
  public:
    /** Konstruktor; 
      Arg1: Abstand vor der Torlinie, an der sich der Roboter positionieren soll
      Arg2: Maximale exzentrische Verschiebung */
    BGoaliePrePenalty (double, double) throw ();
    DriveVector getCmd(const Time&) throw ();
    void gainControl (const Time&) throw (TribotsException);
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void updateTactics (const TacticsBoard&) throw ();

  private:
    SPhysGotoPos goto_pos_skill;
    Vec home;
    double max_excenter;
    double excenter;
    Time last_time_attacker_seen;
  };

}

#endif 
