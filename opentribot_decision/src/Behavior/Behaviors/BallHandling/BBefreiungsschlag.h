#ifndef _BBefreiungsschlag_h_
#define _BBefreiungsschlag_h_

/*
 *  BBefreiungsschlag.h
 *  robotcontrol
 *
 *  Created by Sascha Lange on 13.06.06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "../../Behavior.h"
#include "../../../Fundamental/Time.h"

namespace Tribots
{
  
class BBefreiungsschlag : public Behavior
{
public:
  BBefreiungsschlag(int kickDuration = 80,
										 double probability = 1.);
  virtual ~BBefreiungsschlag() throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();
	
	virtual void updateTactics(const TacticsBoard& tb) throw();
  
  virtual void cycleCallBack(const Time&) throw();
  
protected:
  Time lastActivation;
  int kickDuration;
	double probability;
  bool freeToKick;
  bool hasBall;
  
  Time timeBallEnteredOwnHalf;
  bool isBallInOwnHalf;
};

  
}

#endif
