
#ifndef Tribots_BGoalieOpponentGoalKick_h
#define Tribots_BGoalieOpponentGoalKick_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"

namespace Tribots {

  /** Positioniert den Goalie in der seitlich versetzt neben Tor bei 
      gegnerischer Standardsituation */
  class BGoalieOpponentGoalKick : public Behavior {
  public:
    /** Konstruktor; 
      Arg1: seitlicher Versatz gegenueber Tormitte bei Positionierung,
      Arg2: Abstand zur Torlinie, in der der Roboter sich stellt 
      Arg3: soll Verhalten aktiv sein? */
    BGoalieOpponentGoalKick (double, double, bool) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void gainControl(const Time&) throw(TribotsException);
    void updateTactics (const TacticsBoard&) throw ();  ///< liest "AbstossGegener" = "normal"/"blockTor"

  private:
    SPhysGotoPos goto_pos_skill;
    Time activation_time;
    Time ball_not_seen_in_own_half;
    double offset_y;
    double offset_x;
    
    bool use_for_goal_kick;
    bool use_for_throw_in;  // aber nur bis 1,5 m vor Tor
    bool use_for_free_kick;  // aber nur bis 2,5 m vor Tor oder 1,5m seitlich vor Tor
    Vec relevant_ball;
  };

}

#endif 
