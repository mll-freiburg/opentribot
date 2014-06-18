#ifndef _Tribots_SDribbleBallToPos_H_
#define _Tribots_SDribbleBallToPos_H_

#include "SDribbleBallToPosInterface.h"

namespace Tribots
{
  
  /**
   * Dribbelt den Ball zu einer Position, wobei Hindernisse umfahren werden.
   * Ist aus dem "SLissabonDribbleToXY" Skill hervorgegangen und neigt zu
   * oftmals großzügigen, unvorhersehbaren Schlenkern, in der Regel ohne dabei 
   * den Ball zu verlieren. Funktioniert nur, wenn man den Ball zwischen den
   * Hörnchen hat (gut eingestellte "Dribbelhörnchen" sind absolut notwendig!).
   */
  class SDribbleBallToPos : public SDribbleBallToPosInterface
  {
    public:
      SDribbleBallToPos(double pullBackPositionXPosMax = 370.,
                        double pullBackPositionYPosMax = 350);
      void setParameters(const Vec& target, double transVel, bool forceHardTurn) 
        throw(TribotsException);
      virtual DriveVector getCmd(const Time&) throw(TribotsException);
      
    protected:
      Vec target;
      double transVel;
      bool forceHardTurn;
      
      double pullBackPositionXPosMax;
      double pullBackPositionYPosMax;      
  };
}
#endif
