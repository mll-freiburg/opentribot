#ifndef _BDoubleTeam_H_
#define _BDoubleTeam_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Skills/BasicMovements/SGoToPosEvadeObstaclesOld.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"

#define SKILL SGoToPosEvadeObstaclesOld

namespace Tribots
{
  class BDoubleTeam : public Behavior
  {
  public:
    BDoubleTeam(const std::string name="BDoubleTeam");
    virtual ~BDoubleTeam() throw ();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void updateTactics (const TacticsBoard&) throw ();

  protected:
    SKILL* skill;
    PIDController headingController;
    float transVel;
  };

}
#endif
