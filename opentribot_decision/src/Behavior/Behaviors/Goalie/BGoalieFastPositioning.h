
#ifndef Tribots_BGoalieFastPositioning_h
#define Tribots_BGoalieFastPositioning_h

#include "../../../Behavior/Behavior.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Positioniert den Goalie in Abhaengigkeit der Ballposition und -geschwindigkeit 
      auf einem Kreisbogen vor dem eigenen Tor */
  class BGoalieFastPositioning : public Behavior {
  public:
    /** Konstruktor, uebergeben werden die Homeposition des Goalie (arg1), der linke 
	oder rechte Endpunkt des Goalie-Positionskreisbogens (arg2) sowie die
	maximale Auslenkung des Roboters gegenueber der Ausrichtung zum 
	gegnerischen Tor (arg3) */
    BGoalieFastPositioning (Vec, Vec, Angle, bool) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  private:
    Arc positioning_arc;
    Vec left_end, right_end;
    Vec pa1, pa2;  // Grenzen des Strafraums als Arbeitsbereich des Goalies fuer dieses Verhalten
    Vec goal_post_right;
    Vec goal_post_left;
    Angle max_angle;
    bool use_comm_ball;
  };

}

#endif 
