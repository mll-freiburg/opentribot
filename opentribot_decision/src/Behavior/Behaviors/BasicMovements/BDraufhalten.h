#ifndef _BDRAUFHALTEN_H_
#define _BDRAUFHALTEN_H_

/*
 *  BDraufhalten.h
 *  robotcontrol
 *
 *  Created by Sascha Lange on 01.05.06.
 *  Copyright 2006 University of Osnabrueck. All rights reserved.
 *
 */

#include "BShootImmediately.h"
#include "../../Behavior.h"

namespace Tribots {

/** 
 * Dieses Verhalten implementiert einen (Distanz-)schuss, der relativ leicht
 * ausloest ("einfach mal draufhalten"). Es kann eine Wahrscheinlichkeit 
 * angegeben werden, mit der dieses Verhalten in einer "Ballbesitzphase"
 * zur Anwendung kommen soll. Wird diese z.B. auf 0.1 gesetzt, darf dieses
 * Verhalten ca. jedes 10. Mal, wenn dieser Roboter den Ball neu erhaelt, 
 * ausloesen.
 */
class BDraufhalten : public BShootImmediately
{
public:
  BDraufhalten(double probability, int hackKickLength=45, double maxShootDistance = -1, 
               int cyclesFreeCorridor = 3);
  virtual ~BDraufhalten() throw();
  virtual void updateTactics (const TacticsBoard&) throw ();  
  virtual bool checkInvocationCondition(const Time& t) throw();
  virtual void cycleCallBack(const Time& t) throw();
  
protected:
  double probability;
  double probability_orig;
  bool clearToShoot;
  bool doPossesBallLastTime;
    
  int checkCycles;
  int consecutiveCyclesFree;
  
  double maxShootDistance;
};
  
}


#endif
