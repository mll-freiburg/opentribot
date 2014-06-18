#ifndef _TRIBOTS_BPREOPPONENTSTANDARDSITUATIONNEW_H_
#define _TRIBOTS_BPREOPPONENTSTANDARDSITUATIONNEW_H_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"

namespace Tribots {

  class BPreOpponentStandardSituationNew : public Behavior {
  public:

    BPreOpponentStandardSituationNew(bool blockGoal = false);
    ~BPreOpponentStandardSituationNew() throw();

    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();

    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void gainControl(const Time&) throw(TribotsException);

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

  protected:
    SPhysGotoPosAvoidObstacles* skill;
    bool blockGoal;
    enum { POS_LEFT=1, POS_RIGHT, POS_CENTER };
    int prefside;  // Hysterese fuer bevorzugte Position links/rechts
    Vec ballpos;
    Time ballposknown;
    double mindestballabstand2; // Mindestabstand zum Ball
    double robotspeed; // geschwindigkeit vom Roboter
    bool lookForBall;
    bool noMinDistanceInPA;

    void setBallPos (const Time& t);
    /** berechnet die Positionen der beiden den Ball abschirmenden Spieler */
    Vec getPositionAtBall(const Time& t, int pos, double d = 2000,
                          bool blockGoal = false, bool draw = true);  
    /** berechnet die Positionen der beiden im Block stehenden Spieler */
    Vec getPositionBlock(const Time& t, int pos, double d = 2000,
                         bool blockGoal = false, bool draw = true);
    /** berechnet die Position des Safeties */
    Vec getPositionSafety(const Time& t, double d = 2000, 
                          bool blockGoal = false, bool draw = true);

    const static int toleranz = 100; // Toleranz der Roboterentfernung zum Ball in mm
  };

}

#endif 
