#ifndef _BPreventInterferenceWithPass_H_
#define _BPreventInterferenceWithPass_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/BasicMovements/SBoostToPos.h"

namespace Tribots
{
  class BPreventInterferenceWithPass : public Behavior
  {
  public:
    BPreventInterferenceWithPass(const std::string = "BBPreventInterferenceWithPass");
    virtual ~BPreventInterferenceWithPass() throw ();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);


    virtual void updateTactics (const TacticsBoard&) throw ();

  protected:
    SGoToPosEvadeObstacles* skill; ///< skill zur Positionsanfahrt mit Hindernisausweichen
    float transVel;    
    Time started;
  };

}
#endif
