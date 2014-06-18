#ifndef _BZonePressure_H_
#define _BZonePressure_H_
/*
 *  BZonePressure.h
 *  robotcontrol
 *
 *  Created by Sascha Lange on 08.04.07.
 *  Copyright 2007 University of Osnabrueck. All rights reserved.
 *
 */

#include "../../BDIBehavior.h"

/** ball links:  LEFT: blockGoal, RIGHT: blockMiddle,
*  ball mitte:  LEFT, RIGHT: beide blockGoal
*  ball rechts: LEFT: blockMiddle, RIGHT: blockGoal
*  safety immer blockGoal aber weiter hinten
*/
namespace Tribots {

/** 2-2-1 Zonenpresse mit Doubleteam durch 1. Reihe */ 
class ZonePressure : public BDIBehavior
{
private:
  virtual bool checkConditions(const Time& t) throw();
public:
  ZonePressure();
  virtual bool checkCommitmentCondition(const Time& t) throw();
  virtual bool checkInvocationCondition(const Time& t) throw();
};

}

#endif

