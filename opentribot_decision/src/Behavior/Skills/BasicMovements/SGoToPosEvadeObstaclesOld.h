#ifndef _SGOTOPOSEVADEOBSTACLESOLD_H_
#define _SGOTOPOSEVADEOBSTACLESOLD_H_

#include "../../Skill.h"
#include "../../../Fundamental/PIDController.h"


namespace Tribots
{
  class SGoToPosEvadeObstaclesOld : public Skill 
  {
  public:
    SGoToPosEvadeObstaclesOld();
    virtual ~SGoToPosEvadeObstaclesOld() throw();
    
    void init(Vec targetPos,
	      double driveVel,
	      Vec targetHeading,
	      bool avoidball=0) throw(TribotsException);
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
  
  protected:
    Vec targetPos;
    double driveVel;
    Vec targetHeading;
    bool avoidBall;
    
    PIDController headingController;
  };
}

#endif
