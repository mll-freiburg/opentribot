#ifndef _BStuckDistanceShooter_H_
#define _BStuckDistanceShooter_H_

#include "../../Behavior.h"

namespace Tribots
{
  class BStuckDistanceShooter : public Behavior
  {
  public:
    BStuckDistanceShooter();
    virtual ~BStuckDistanceShooter() throw();
    
    virtual DriveVector getCmd(const Time& tt) throw(TribotsException);
    
    virtual void gainControl(const Time& t) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();
    
  protected:
    Time starttime;
    double rotation;
  };
}
#endif
