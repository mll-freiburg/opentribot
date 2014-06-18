#ifndef _Tribots_SDribbleBallToPosInterface_H_
#define _Tribots_SDribbleBallToPosInterface_H_

#include "../../Skill.h"

namespace Tribots
{
  
  /**
   * Dribbelt den Ball zu einer Position, wobei Hindernisse nicht umfahren werden.
   */
  class SDribbleBallToPosInterface : public Skill
  {
    public:
      SDribbleBallToPosInterface(const char * name) : Skill(name){ ; }
      virtual void setParameters(const Vec& target, double transVel, bool forceHardTurn) 
        throw(TribotsException)=0;
  };
}

#endif
