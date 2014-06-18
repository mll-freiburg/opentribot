#ifndef _Tribots_BDribbleBallToPassPosition_H_
#define _Tribots_BDribbleBallToPassPosition_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{

/** 
 * Dribbelt den Ball zum Tor und weicht dabei Hindernissen aus. Neigt zu
 * großzügigen Schlenkern (siehe auch SDribbleBallToPos).
 */
class BDribbleBallToPassPosition : public Tribots::Behavior
{
public:
  BDribbleBallToPassPosition(double transVel,
			     const Quadrangle &decisionArea,
			     double passProbability);
	virtual ~BDribbleBallToPassPosition() throw();
  
  /** Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Roboter hat den Ball */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  void cycleCallBack (const Time& t) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);   
  bool m_bDribblingToSecondPoint;
  bool m_bMessageSend;
  bool m_bDecisionMade;
  bool m_bDoPass;
  int m_iLostBallLoops;
  
protected:
  double transVel;
  Quadrangle decisionArea;
  double passProbability;
  SDribbleBallToPos* skill;
};

};

#endif //_Tribots_BDribbleBallToPassPosition_H_
