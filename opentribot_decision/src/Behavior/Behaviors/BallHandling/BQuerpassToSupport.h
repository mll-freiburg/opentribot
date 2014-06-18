#ifndef _BQuerpassToSupport_h_
#define _BQuerpassToSupport_h_

/*
 *  BQuerpassToSupport.h
 *  robotcontrol
 *
 *  Created by Sven Kerkling on 25.04.07.
 *
 */

#include "../../Behavior.h"
#include "../../../Fundamental/Time.h"

namespace Tribots
{
  
class BQuerpassToSupport : public Behavior
{
public:
  BQuerpassToSupport(int shortKickDuration = 30,
										 double passProbability = 1.,
                     bool passOnlyIfGoalBlocked  = false);
  virtual ~BQuerpassToSupport() throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();
	
	virtual void updateTactics(const TacticsBoard& tb) throw();
  
  virtual void cycleCallBack(const Time&) throw();
  virtual void loseControl(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  
protected:
  Time lastActivation;
  int shortKickDuration;
  bool kicked;
  double passProbability;
  int incProbLead;    ///< the minimal lead necessary, before behavior starts to pass more often
  bool freeToPass;
  bool passOnlyIfGoalBlocked;
  Angle oldheading; 
  bool hasBall;
  bool turned;
  int seite;
};

  
}

#endif
