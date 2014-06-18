#ifndef _TRIBOTS_BPOSTOPPONENTSTANDARDSITUATION_H_
#define _TRIBOTS_BPOSTOPPONENTSTANDARDSITUATION_H_

#include "../../Behavior.h"

namespace Tribots {

  class BPostOpponentStandardSituation : public Behavior {
  public:

    BPostOpponentStandardSituation();
    ~BPostOpponentStandardSituation() throw();

    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    
    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void gainControl(const Time&) throw(TribotsException);
    
    virtual void loseControl(const Time&) throw(TribotsException);
    virtual void cycleCallBack(const Time&) throw() ;

  protected:
    int waitingTime;
    Time tLosSend;
    bool communicateBallMoved;
    Time lastBallMoved;
    int lastGameState;
  };

}

#endif 
