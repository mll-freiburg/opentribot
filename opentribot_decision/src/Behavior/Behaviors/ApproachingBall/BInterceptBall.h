
#ifndef Tribots_BInterceptBall_h
#define Tribots_BInterceptBall_h

#include "../../../Behavior/Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"

namespace Tribots {

  /** "Verteidigt" die Seitenline, indem er sich auf einen Schnittpunkt mit der
      Ballbewegung stellt. */
  class BInterceptBall : public Behavior {
  public:
    BInterceptBall () throw ();
    ~BInterceptBall() throw() {};
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();

  protected:
    PIDController headingController;
    PiecewiseLinearFunction breakF;
  };

}

#endif 
