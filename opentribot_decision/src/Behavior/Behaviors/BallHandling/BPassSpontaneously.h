#ifndef _BPassSpontaneously_h_
#define _BPassSpontaneously_h_

/*
 *  BPassSpontaneously.h
 *  robotcontrol
 *
 *  Created by Sascha Lange on 15.05.06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "../../Behavior.h"
#include "../../../Fundamental/Time.h"

// TOOD: 1. Winkel des Empfängers überprüfen (heading). 2. Checken, ob Weg Zum Tor komplett frei und Weg nach vorne für 1000mm und 800mm Breite frei-> dann nicht passen (Ein und Ausstellbar über Taktikboard)

namespace Tribots
{
  
class BPassSpontaneously : public Behavior
{
public:
  BPassSpontaneously(int shortKickDuration = 30, int longKickDuration = 40,
										 double passProbability = 1.,
                     bool passOnlyIfGoalBlocked  = false);
  virtual ~BPassSpontaneously() throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();
	
	virtual void updateTactics(const TacticsBoard& tb) throw();
  
  virtual void cycleCallBack(const Time&) throw();
  virtual void loseControl(const Time&) throw(TribotsException);
  
protected:
  Time lastActivation;
  int shortKickDuration;
  int longKickDuration;
  Vec target;
  unsigned int targetId;
  bool kicked;
	double passProbability;
  int incProbLead;    ///< the minimal lead necessary, before behavior starts to pass more often
  bool freeToPass;
  bool passOnlyIfGoalBlocked;
  
  bool hasBall;
};

  
}

#endif
