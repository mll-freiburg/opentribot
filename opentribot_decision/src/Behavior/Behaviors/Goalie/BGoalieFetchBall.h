
#ifndef Tribots_BGoalieFetchBall_h
#define Tribots_BGoalieFetchBall_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoBallCarefully.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Faehrt hinter den Ball, um ihn aus dem Tor oder aus Tornaehe herauszuholen */
  class BGoalieFetchBall : public Behavior {
  public:
    /** Konstruktor; arg1: seitlicher Abstand der Fetching Area Grenze vom Torpfosten in mm */
    BGoalieFetchBall (double) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void gainControl(const Time&) throw();

  private:
    Vec pa1, pa2;  // Grenzen des Strafraums
    Vec fa1, fa2;  // Grenze Fetching area
    Vec rp1, rp2;  // zwei Referenzpositionen zur Berechnung der unterschiedlichen Richtungsbereiche
    Vec goal_post_right;
    Vec goal_post_left;
    int latest_dir;
    SPhysGotoBallCarefully goto_ball_skill;
  };

}

#endif 
