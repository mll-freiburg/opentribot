
#ifndef Tribots_BCatchBall_h
#define Tribots_BCatchBall_h

#include "../../../Behavior/Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"

namespace Tribots {

  /** "Verteidigt" die Seitenline, indem er sich auf einen Schnittpunkt mit der
      Ballbewegung stellt. */
  class BCatchBall : public Behavior {
  public:
    BCatchBall () throw ();
    ~BCatchBall() throw() {};
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  protected:
    PiecewiseLinearFunction breakF;
    
    PiecewiseLinearFunction goToBall;
  };

}

#endif 
