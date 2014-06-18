#ifndef _BFARSHOOT_H_
#define _BFARSHOOT_H_

/*
 *  BFarShoot.h
 *  robotcontrol
 *
 *  Created by Roland Hafner
 *  Copyright 2006 University of Osnabrueck. All rights reserved.
 *
 */

#include "../../Behavior.h"

namespace Tribots {

class BFarShoot : public Tribots::Behavior
{
public:
  BFarShoot(double probability);
  virtual ~BFarShoot() throw();
  virtual void updateTactics (const TacticsBoard&) throw ();  
  virtual bool checkInvocationCondition(const Time& t) throw();
  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual void cycleCallBack(const Time& t) throw();
  virtual DriveVector getCmd(const Time&) throw(TribotsException);

protected:
  double probability;
  int    duration;

  bool   m_bDecisionMade;
  bool   mayBecomeActive;
  
  int activationLevel;
  enum { ALWAYS, IF_PRESSURED, OPP_IN_FUNNEL, OPP_IN_FRONT, NEVER };
};
  
}


#endif
