
#ifndef Tribots_BTurnAroundPos_h
#define Tribots_BTurnAroundPos_h

#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include "../../../Fundamental/PIDController.h"

namespace Tribots {

  /** GoalKick oder FreeKick im eigenen Strafraum durchfuehren */
  class BTurnAroundPos : public Behavior {
  public:
    /** Konstruktor; Argumente sind: arg1: maximaler Ausdrehwinkel, arg2: Torwartfluegel?, arg3: kick_permission */
    BTurnAroundPos() throw ();
    ~BTurnAroundPos() throw ();
    DriveVector getCmd(const Time&) throw ();
    /** preOwnGoalKick oder preOwnFreeKick und Ball im Strafraum */
    bool checkInvocationCondition (const Time&) throw ();
    /** Invocation Bedingung oder war aktiv und freePlay */
    bool checkCommitmentCondition (const Time&) throw ();
    void gainControl(const Time&) throw();
    void loseControl(const Time&) throw();

  private:
    bool isKickingGood(Angle robotHeading, Angle targetHeading, const Time& t);
    //SApproachMovingBall* skill;
    //SApproachParkedBall* skill;
    SDribbleBallToPos* skill;
    PIDController headingController;

    bool was_active;
    Angle target_direction;
    bool target_direction_known;
    double kicker_half_width;
    double kick_distance;
    double turn_radius;
    Vec ballpos;       // aktuell angenommene Ballposition
    int wait_rel_to_goal;  // -1: warte ausserhalb des Tores, +1: warte innerhalb, 0: dontcare
    int dir;
    bool was_moving_around_goal_post;

    Vec pa1, pa2;   // Ecken der Penalty area
    Vec goal_post_right;
    Vec goal_post_left;
    Vec goal_post_mid;
    double robotball_radius;

    // Bereiche fuer Ballposition, nach denen unterschieden wird:
    Triangle dont_care_direction_area;
    XYRectangle near_goal_post_inner_area;
    XYRectangle near_goal_post_inner_area_dil;
    XYRectangle near_goal_post_outer_area;
    XYRectangle near_goal_post_outer_area_dil;

    const static float DOGCURVEFACTOR = 0.55;
    const static float DOGDISTANCE = 350;
  };

}

#endif 
