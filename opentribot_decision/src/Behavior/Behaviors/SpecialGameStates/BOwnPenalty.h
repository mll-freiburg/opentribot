#ifndef _TRIBOTS_BPREPENALTY_H_
#define _TRIBOTS_BPREPENALTY_H_

#include "../../Behavior.h"
#include "../../SPBehavior.h"
#include "../../Skills/Goalie/SPhysGotoBallCarefully.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"

namespace Tribots {

  /** Behavior fuer die GameStates PreOwnPenalty und OwnPenalty.
      Stellt sich in der Mitte auf und faehrt nach Start des Penaltys
      zum Ball, entscheidet sich fuer eine Ecke, stellt sich zum Schuss
      auf, nimmt Anlauf und haut die Kirsche hoffentlich in den Kasten */
  class BOwnPenalty : public SPBehavior {
    double targetX;
  public:
    BOwnPenalty();
    ~BOwnPenalty() throw() {;}
    bool checkCommitmentCondition(const Time&) throw();
    bool checkInvocationCondition(const Time&) throw();
  };

/* // Alte Version:
  class BOwnPenalty : public Behavior {
  public:

    BOwnPenalty();
    ~BOwnPenalty() throw();

    bool checkCommitmentCondition(const Time&) throw();
    bool checkInvocationCondition(const Time&) throw();
    DriveVector getCmd(const Time&) throw(TribotsException);
    void gainControl(const Time&) throw();
    void loseControl(const Time&) throw();
    void updateTactics (const TacticsBoard& tb) throw ();

  protected:
    SPhysGotoBallCarefully* goto_ball;
    SPhysGotoPos* goto_pos;
    SPhysGotoPos* goto_pos_slowly;
    SPhysGotoPosAvoidObstacles* goto_pos_obs;

    bool was_active;
    bool decision_done;
    bool point_of_no_return;
    bool ball_touched;
    Time wait;
    Vec target_kick_pos;
    Vec orig_ball_pos;
    double shoot_height;
  }; */

}

#endif 
