
#ifndef Tribots_BGoalieRaisedBall_h
#define Tribots_BGoalieRaisedBall_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/RingBuffer.h"

namespace Tribots {

  /** Positioniert den Goalie in Abhaengigkeit der Ballposition bei angehobenem Ball:
      1. wenn Ball ausserhalb des Feldes war: bleibe stehen
      2. wenn nicht, fahre in Tormitte */
  class BGoalieRaisedBall : public Behavior {
  public:
    BGoalieRaisedBall () throw ();
    DriveVector getCmd(const Time&) throw ();
    void cycleCallBack(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void gainControl (const Time&) throw ();
    void loseControl (const Time&) throw ();

  private:
    struct BallPos {
      Vec pos;
      Vec vel;
      unsigned long int cycle;
    };

    Vec goal_center;
    double goal_half_width;
    double robot_half_width;
    RingBuffer<BallPos> oldball;
    Vec target_pos;
    Time time_of_start_raised;
    bool was_active;

    SPhysGotoPos goto_pos_skill;
  };

}

#endif 
