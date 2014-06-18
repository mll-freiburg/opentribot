#ifndef _BProtectGoal_H_
#define _BProtectGoal_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Skills/BasicMovements/SGoToPosEvadeObstaclesOld.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"

#define SKILL SGoToPosEvadeObstaclesOld

namespace Tribots
{
  /** deckt den langen Pfosten, steht also in Richtung der Spielfeldmitte 
   *  hinter dem Doubleteam */
  class BProtectGoal : public Behavior
  {
  public:
    BProtectGoal(bool obeyPenaltyAreaRules=true, std::string name="BProtectGoal");
    virtual ~BProtectGoal() throw ();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void updateTactics (const TacticsBoard&) throw ();

  protected:
    SKILL* skill;
    PIDController headingController;
    bool obeyPenaltyAreaRules;
    float transVel;
    Vec protectPos;  // bei aufruf merken, ob linken oder rechten torpfosten decken
    double minDistanceToBall;
    double maxDistanceToBall;
  };

}
#endif
