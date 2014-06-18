
#ifndef Tribots_BGoaliePenaltyAttackBall_h
#define Tribots_BGoaliePenaltyAttackBall_h

#include "BGoalieAttackBall.h"


namespace Tribots {

  /** Faehrt den Ball an und schiesst ihn weg, wenn der Roboter sich hinter dem Ball befindet 
      wie BGoalieAttackBall, aber mit groesserem Aktionsbereich; wird nur bei Penalty aktiv */
  class BGoaliePenaltyAttackBall : public BGoalieAttackBall {
  public:
    /** Konstruktor; arg1: sollen Hindernisse beruecksichtigt werden? arg2 und arg3: Eckpunkte der Attack-Area, arg4: Schiessen erlaubt? */
    BGoaliePenaltyAttackBall (bool, Vec, Vec, bool) throw ();
    /** Invocation Bedingung: Roboter befindet sich hinter dem Ball und ist auf ihn ausgerichtet, Ball befindet sich im Angriffsbereich, Penalty */
    bool checkInvocationCondition (const Time&) throw ();
    /** Commitment Condition: wie Invocation Bedingung */
    bool checkCommitmentCondition (const Time&) throw ();
  };

}

#endif 
