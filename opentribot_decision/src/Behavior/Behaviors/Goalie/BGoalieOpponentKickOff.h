
#ifndef Tribots_BGoalieOpponentKickOff_h
#define Tribots_BGoalieOpponentKickOff_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"

namespace Tribots {

  /** Positioniert den Goalie neben dem Tor */
  class BGoalieOpponentKickOff : public Behavior {
  public:
    /** Konstruktor; 
      Arg1: soll Verhalten aktiv sein? */
    BGoalieOpponentKickOff (bool) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
//    void gainControl(const Time&) throw(TribotsException);
    void loseControl(const Time&) throw(TribotsException);
    void updateTactics (const TacticsBoard&) throw ();  ///< liest "GoalieKickOff" = "Zentral"/"Seitlich"
    void cycleCallBack(const Time&) throw();

  private:
    SPhysGotoPosAvoidObstacles goto_pos_skill;
    Time activation_time;
    Time ball_not_seen_in_own_half;

    bool use_behavior;
    bool dynamic_mode;
    bool dynamic_on;
    bool was_on;
    Time time_free_started;
    unsigned int time_of_free;
    unsigned int opponent_score;
  };

}

#endif 
