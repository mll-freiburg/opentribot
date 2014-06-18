#ifndef _BSupportLongPass_H_
#define _BSupportLongPass_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/BasicMovements/SBoostToPos.h"

namespace Tribots
{
  class BSupportLongPass : public Behavior
  {
  public:
    BSupportLongPass(const std::string = "BBSupportLongPass");
    virtual ~BSupportLongPass() throw ();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void gainControl(const Time&) throw(TribotsException);

    virtual void updateTactics (const TacticsBoard&) throw ();

  protected:
    SGoToPosEvadeObstacles* skill; ///< skill zur Positionsanfahrt mit Hindernisausweichen
    SBoostToPos*       fast_skill; ///< skill zur schnellen Positionsanfahrt, keine Feinregelung
    Skill*             activeSkill; ///< aktuell verwendeter Skill

    enum { BLOCK_LEFT=0, BLOCK_RIGHT };
    int blockPosition;
    float transVel;
    
    enum { TACTICS_DEFENSIVE, TACTICS_NEUTRAL, TACTICS_OFFENSIVE };
    int presentTactics;
    bool boostAllowed;
  };

}
#endif
