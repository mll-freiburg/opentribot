
#ifndef Tribots_BGoalieAttackBall_h
#define Tribots_BGoalieAttackBall_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"

namespace Tribots {

  /** Faehrt den Ball an und schiesst ihn weg, wenn der Roboter sich hinter dem Ball befindet */
  class BGoalieAttackBall : public Behavior {
  public:
    /** Konstruktor; arg1: sollen Hindernisse beruecksichtigt werden? arg2 und arg3: Eckpunkte der Attack-Area, arg4: Schiessen erlaubt?, arg5: Anfahrskill, das benutzt werden soll */
    BGoalieAttackBall (bool, Vec, Vec, bool, SPhysGotoPos* =NULL) throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw ();
    /** Invocation Bedingung: Roboter befindet sich hinter dem Ball  
     * und ist auf ihn ausgerichtet, Ball befindet sich im Angriffsbereich */
    bool checkInvocationCondition (const Time&) throw ();
    /** Commitment Condition: wie Invocation Bedingung */
    bool checkCommitmentCondition (const Time&) throw ();
    void updateTactics (const TacticsBoard&) throw ();  ///< liest "GoalieAttackArea" = X (mm)
    void gainControl (const Time&) throw(TribotsException);

  private:
    Vec goal_post_right;
    Vec goal_post_left;
    Vec aa1, aa2;  // Ecken der Attack-Area
    double robot_half_width;
    double kick_distance;
    double kicker_half_width;
    double ball_radius;
    SPhysGotoPos own_goto_pos_skill;
    SPhysGotoPos* goto_pos_skill;

    bool consider_obstacles;
    bool kick_permission;
  };

}

#endif 
