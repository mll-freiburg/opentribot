#ifndef _BStuckOwnsBall_H_
#define _BStuckOwnsBall_H_

#include "../../Behavior.h"

namespace Tribots
{
  class BStuckOwnsBall : public Behavior
  {
  public:
    BStuckOwnsBall();
    virtual ~BStuckOwnsBall() throw();
    
    virtual DriveVector getCmd(const Time& tt) throw(TribotsException);
    
    virtual void gainControl(const Time& t) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();
    
  protected:
    Time starttime;
    Vec relTarget;
    double rotation;
  };
}
#endif
