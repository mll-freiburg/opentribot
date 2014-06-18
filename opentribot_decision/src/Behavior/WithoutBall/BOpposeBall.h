#ifndef Tribots_BOpposeBall_h
#define Tribots_BOpposeBall_h

#include "../../../Behavior/Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"

namespace Tribots {

  /** "Verteidigt" die Seitenline, indem er sich auf einen Schnittpunkt mit der
      Ballbewegung stellt. */
  class BOpposeBall : public Behavior {
  public:
    /** contructor
     *  @param hearToSignal hearing to communicated signal can be deactivated for testing purpose
     */
    BOpposeBall (bool hearToSignal = true) throw ();
    ~BOpposeBall() throw() {};
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    virtual void gainControl(const Time&) throw(TribotsException);

  protected:
    PIDController headingController;
    Time tActivated;
    bool hearToSignal;
  };

}

#endif 
