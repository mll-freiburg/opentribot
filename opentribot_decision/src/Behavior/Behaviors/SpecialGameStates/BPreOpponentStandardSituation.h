#ifndef _TRIBOTS_BPREOPPONENTSTANDARDSITUATION_H_
#define _TRIBOTS_BPREOPPONENTSTANDARDSITUATION_H_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"

namespace Tribots {

  class BPreOpponentStandardSituation : public Behavior {
  public:

    BPreOpponentStandardSituation(bool blockGoalOppGoalKick = false);
    ~BPreOpponentStandardSituation() throw();

    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();
    
    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void gainControl(const Time&) throw(TribotsException);

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

  protected:
    SPhysGotoPosAvoidObstacles* skill;
    bool blockGoal;
    Vec getPositionAtBall(const Time& t, double d = 2000,
                bool blockGoalOppGoalKick = false);  // d=Mindestabstand zum Ball
    Vec getPositionBlock(const Time& t, int pos, double d = 2000,
                bool blockGoalOppGoalKick = false);
    Vec getPositionSafety(const Time& t, double d = 2000);
    enum { POS_LEFT=1, POS_RIGHT };
    int prefside;  // Hysterese fuer bevorzugte Position links/rechts
    Angle targetEvadeDir;
    unsigned int targetEvadeDirArea;
    Vec ballpos;
    Time ballposknown;
    double mindestballabstand1; // Mindestabstand zum Ball
    double mindestballabstand2; // Mindestabstand zum Ball
    void setTargetEvadeDirection (Vec tg, Vec tghd, int prefside, bool is_ball);
    void setBallPos (const Time& t);
    double robotspeed; // geschwindigkeit vom Roboter
    bool lookForBall;

    const static int toleranz = 100; // Toleranz der Roboterentfernung zum Ball in mm
  };

}

#endif 
