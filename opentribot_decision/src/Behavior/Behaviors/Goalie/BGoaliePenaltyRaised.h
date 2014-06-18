
#ifndef _Tribots_BGoaliePrePenalty_h_
#define _Tribots_BGoaliePrePenalty_h_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"

namespace Tribots {

  /** Faehrt den Roboter bei raisedBall und opponentPenalty per Zufall in eine Tor-Ecke */
  class BGoaliePenaltyRaised : public Behavior {
  public:
    /** Konstruktor; uebergeben werden die Wahrscheinlichkeiten fuer:
      arg1: in die linke Ecke fahren, arg2: in der Mitte bleiben, arg3: in die rechte Ecke fahren */
    BGoaliePenaltyRaised (double, double, double) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void gainControl(const Time&) throw ();
    void cycleCallBack(const Time&) throw ();
    
  private:
    double targetx;
    double pleft, pmiddle, pright;
    Vec ball1, ball2, ball3, ball4, ball5;
  };

}

#endif 
