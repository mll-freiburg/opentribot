#ifndef _BEigenMove_H_
#define _BEigenMove_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPosInterface.h"

namespace Tribots
{
  class BEigenMove : public Behavior
  {
  public:
    BEigenMove(Vec targetPosRelToOppGoal = Vec(0., -2000));
    ~BEigenMove() throw();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void gainControl(const Time&) throw(TribotsException);

    virtual void updateTactics (const TacticsBoard&) throw ();  

  protected:
      SDribbleBallToPosInterface* skill;
      Vec targetPos;
      double transVel;
  };
}
#endif
