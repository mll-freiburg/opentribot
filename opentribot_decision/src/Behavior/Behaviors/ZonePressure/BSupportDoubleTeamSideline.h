#ifndef _BSupportDoubleTeamSideline_H_
#define _BSupportDoubleTeamSideline_H_

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
  class BSupportDoubleTeamSideline : public Behavior
  {
  public:
    BSupportDoubleTeamSideline(std::string name="BSupportDoubleTeamSideline");
    virtual ~BSupportDoubleTeamSideline() throw ();

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
