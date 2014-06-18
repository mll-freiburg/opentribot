#ifndef _BPASSBEFOREGOAL_H_
#define _BPASSBEFOREGOAL_H_

#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/BallHandling/SPass.h"

namespace Tribots
{
  class BPassBeforeGoal : public Behavior
  {
  public:
    BPassBeforeGoal(int kickDuration, double passProbability = 0.);
    ~BPassBeforeGoal() throw();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);
    virtual void updateTactics (const TacticsBoard&) throw ();  
		virtual void cycleCallBack(const Time& t) throw();
    bool calculateTarget(const Time& t);

  protected:
    SPass* skill;
    Vec targetPos;
    unsigned int targetId;
    double passProbability;
    double transVel;
		
		int lostcounter;
		bool freeToPass;
		bool hasBall;
    // the minimal lead necessary, before behavior starts to pass more often
		int incProbLead;
		bool alreadyCommunicated;
    int kickDuration;
    
    bool waitPhase;
    Time waitSince;
    
    int messageID;
    int sendCounter;
    
		Quadrangle allowedPositionLeft;
		Quadrangle allowedPositionRight;
  };
}
#endif
