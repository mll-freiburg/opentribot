#ifndef _SGOTOPOSEVADEOBSTACLES_H_
#define _SGOTOPOSEVADEOBSTACLES_H_

#include "../../Skill.h"
#include "../Goalie/SPhysGotoPosAvoidObstacles.h"


namespace Tribots
{
  class SGoToPosEvadeObstacles : public Skill 
  {
  public:
    SGoToPosEvadeObstacles();
    virtual ~SGoToPosEvadeObstacles() throw();
    
    void set_target_evade_strategy (Angle) throw ();
    void force_target () throw ();
    void init(Vec targetPos,
              double driveVel,
              Vec targetHeading,
              bool avoidball=0) throw(TribotsException);
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
  
  protected:
    SPhysGotoPosAvoidObstacles* physgoto;
  };
}

#endif
