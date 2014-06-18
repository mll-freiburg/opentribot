
#ifndef Tribots_BGoaliePenaltyInterceptBall_h
#define Tribots_BGoaliePenaltyInterceptBall_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"

namespace Tribots {

  /** Positioniert sich auf der Linie; wird nur bei Penalty aktiv */
  class BGoaliePenaltyInterceptBall : public Behavior {
  public:
    /** Konstruktor */
    BGoaliePenaltyInterceptBall () throw ();
    /** Invocation Bedingung: Penalty */
    bool checkInvocationCondition (const Time&) throw ();
    /** Commitment Condition: wie Invocation Bedingung */
    bool checkCommitmentCondition (const Time&) throw ();
    
    void gainControl (const Time&) throw (TribotsException);
    DriveVector getCmd(const Time&) throw ();
  private:
    SPhysGotoPos goto_pos_skill;    
  };

}

#endif 
