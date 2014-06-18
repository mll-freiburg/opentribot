#ifndef _Tribots_BTouchBallAfterStandard_H_
#define _Tribots_BTouchBallAfterStandard_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include "../../../Fundamental/geometry.h"
#include "../ApproachingBall/BApproachBallFromBehindPointingToMiddle.h"
namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * großzügigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BTouchBallAfterStandard : public Tribots::Behavior
{
public:
  BTouchBallAfterStandard();
	virtual ~BTouchBallAfterStandard() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  void cycleCallBack (const Time& t) throw();
  virtual void updateTactics(const TacticsBoard& tb) throw();
 
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  BApproachBallFromBehindPointingToMiddle* goToBall;  
protected:
  bool opponentCanExecuteStandard;
  int counter;
  int counter2;
  int lastGameState;
  bool passed;
  Time start;
};

};

#endif //_Tribots_BTouchBallAfterStandard_H_
