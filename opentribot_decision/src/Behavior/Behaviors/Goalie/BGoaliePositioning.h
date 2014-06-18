
#ifndef Tribots_BGoaliePositioning_h
#define Tribots_BGoaliePositioning_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Positioniert den Goalie in Abhaengigkeit der Ballposition und -geschwindigkeit 
      auf einem Kreisbogen vor dem eigenen Tor */
  class BGoaliePositioning : public Behavior {
  public:
    /** Konstruktor, uebergeben werden die Homeposition des Goalie (arg1), der linke 
        oder rechte Endpunkt des Goalie-Positionskreisbogens (arg2) sowie die
        maximale Auslenkung des Roboters gegenueber der Ausrichtung zum 
        gegnerischen Tor (arg3), arg4: Anfahrskill */
    BGoaliePositioning (Vec, Vec, Angle, bool, SPhysGotoPos* =NULL) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void gainControl (const Time&) throw(TribotsException);

  private:
    Arc positioning_arc;
    Vec left_end, right_end;
    Vec pa1, pa2;  // Grenzen des Strafraums als Arbeitsbereich des Goalies fuer dieses Verhalten
    Vec goal_post_right;
    Vec goal_post_left;
    Angle max_angle;
    SPhysGotoPos own_goto_pos_skill;
    SPhysGotoPos* goto_pos_skill;
    bool use_comm_ball;
  };
  
  /** Wie BGoaliePositioning, nur auf einem anderen Kreisbogen naeher am
      Tor und nur, solange der Ball weit entfernt ist */
  class BGoaliePositioningFarBall : public BGoaliePositioning {
  public:
    /** Argumente wie beim Konstruktor von BGoaliePositioningFarBall */
    BGoaliePositioningFarBall (Vec, Vec, Angle, bool, SPhysGotoPos* =NULL) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void cycleCallBack (const Time&) throw ();
  private:
    XYRectangle ballFarArea;  ///< Bereich des Feldes, in dem ein entfernter Ball gesehen wird
    Time ballFarSeen;   ///< letzter Zeitpunkt, zu dem der Ball in ballFarArea war
  };
  
}

#endif
