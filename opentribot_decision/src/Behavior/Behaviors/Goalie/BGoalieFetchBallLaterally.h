
#ifndef Tribots_BGoalieFetchBallLaterally_h
#define Tribots_BGoalieFetchBallLaterally_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoBallCarefully.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Faehrt hinter den Ball, um ihn aus dem Tor herauszuholen,
   wenn der Ball in der Naehe der Torpfosten liegt, der Roboter also
   nicht vollstaendig hinter ihn fahren kann */
  class BGoalieFetchBallLaterally : public Behavior {
  public:
    /** Konstruktor; Arg1: Groesse der Fetching Area vom Torpfosten nach aussen, Arg2: maximaler Ausdrehwinkel, Arg3: Torwart mit Fluegeln? */
    BGoalieFetchBallLaterally (double, Angle, bool) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Quadrangle fetching_area_right;  // rechtsseitiger Teil der Fetching area
    Vec pa1, pa2;
    Vec goal_post_right;
    Vec goal_post_left;
    Angle max_angle;
    bool has_goalie_wings;
    double ball_radius;
    double min_robot_radius;
    SPhysGotoPos goto_pos_skill;
    SPhysGotoBallCarefully goto_ball_skill;
  };

}

#endif 
