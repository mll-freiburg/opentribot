#ifndef _BPassChallenge_h_
#define _BPassChallenge_h_

/*
 *  BPassChallenge.h
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
  
class BPassChallenge : public Behavior
{
public:
  BPassChallenge(int shortKickDuration = 40, int longKickDuration = 50,
										 double passProbability = 1.,
                     bool passOnlyIfGoalBlocked  = false);
  virtual ~BPassChallenge() throw();
  
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
