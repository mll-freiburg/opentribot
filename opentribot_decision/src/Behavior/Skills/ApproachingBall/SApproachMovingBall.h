#ifndef _SApproachMovingBall_H_
#define _SApproachMovingBall_H_

#include "../../Skill.h"
#include "../../../Fundamental/PIDController.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"

namespace Tribots
{
  /**
   * Fährt einen sich bewegenden Ball an. Der Skill besitzt hierzu zwei 
   * Parameter, die vor einem Aufruf von getCmd gesetzt werden müssen:
   * targetdir gibt an, in welcher
   * Richtung der Ball "durchfahren" werden soll und desiredVel bestimmt
   * die Anfahrtgeschwindigkeit. Zu beachten ist, dass es sich bei der
   * Geschwindigkeitsangabe um einen Zielwert handelt, der sowohl 
   * unterschritten (Ball sehr nahe) als auch überschritten (Ball rollt 
   * schnell) werden kann. Die im Konstruktor angegebene maxVel wird
   * unter keinen Umständen überschritten.
   */
  class SApproachMovingBall : public Skill
  {
  public:
    /**
     * Konstruiert einen Skill. Als Parameter kann die maximal erlaubte
     * Geschwindigkeit angegeben werden.
     */
    SApproachMovingBall() throw();
//    SApproachMovingBall(double maxVel = 3.0) throw();
    
    virtual ~SApproachMovingBall() throw() {};
    /** 
     * Setzt die beiden Parameter des Skills.
     */
    virtual void setParameters(const Vec& targetdir,
			       double desiredVel) throw(TribotsException);
    virtual void setParameters(const Vec& targetDriveDir,
                  const Vec& targetHeading,
                               double desiredVel) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);
    
    DriveVector getCmd(const Time&) throw(TribotsException);
    virtual void updateTactics(const TacticsBoard& tb) throw();
  protected:
    Vec approachFacing;
    Vec approachDirection;
    Vec approachFacing_exp_smoothed;  
    PIDController headingController;
    PiecewiseLinearFunction approachTarget;
    PiecewiseLinearFunction approachSpeeds;
    PiecewiseLinearFunction lowVelMod;
    double maxVel;          ///< maximal zulässige Fahrgeschwindigkeit
    double desiredVel;      ///< gewünschte Fahrgeschwindigkeit
    bool ausweichennaheball;
    bool active;
  };
}
#endif
