
#ifndef Tribots_BInterceptBallStatic_h
#define Tribots_BInterceptBallStatic_h
#include <cmath>
#include "../../../Behavior/Behavior.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Fundamental/PIDController.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"

namespace Tribots {

  class BInterceptBallStatic : public Behavior {
  public:
    BInterceptBallStatic () throw ();
    ~BInterceptBallStatic() throw() {};
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    virtual void cycleCallBack (const Time &t) throw ();
    /**
     * erkennt einen auf den Roboter zurollenden Ball der gepasst sein koennte
     */
    bool detectPassedBall (const Time&) throw ();
    virtual void loseControl(const Time& t) throw (TribotsException) { isActive = false; }
    
  protected:
    PiecewiseLinearFunction breakF;
    PiecewiseLinearFunction goToBall;
    PiecewiseLinearFunction rotateFactor;
    PiecewiseLinearFunction intersectSpeed;
    const static unsigned int LOOKAHEAD = 3;
    const static double MAX_ROTATE_SPEED; // really the max
    const static double MAX_INTERSECT_SPEED; 
    unsigned int cycle;
    bool invocableCycle[LOOKAHEAD];
  private:
    bool checkConditions (const Time&) throw ();
    bool isActive;
  };

}

#endif 
