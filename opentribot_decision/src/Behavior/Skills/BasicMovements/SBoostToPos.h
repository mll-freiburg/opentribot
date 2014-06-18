#ifndef _S_BOOST_TO_POS_H_
#define _S_BOOST_TO_POS_H_

#include "../../Skill.h"

namespace Tribots
{
  /** Versucht durch maximale Fahrgeschwindigkeit in y-Richtung den Roboter
   * mit maximaler Geschwindigkeit mit Ball auf Position zu fahren
   * - vordere zwei Räder auf max. vel
   * - hinteres Rad zur Ausrichtungskorrektur
   * ! check conditions abfragen, funktioniert nur dann
   * Parameter:
   * mindist: Abstand wann skill Ziel als erreicht ansieht in mm (kleine Abstände gehen nicht->SGoToPos)
   * withBall: fahre mit Ball, nur eine Richtung zugelassen
   * max_abs_angle_diff: wenn mit Ball, bis zu welchem Winkel soll Skill angehen
   **/
  class SBoostToPos : public Skill
  {
  public:
    SBoostToPos(double _mindist = 1000.0 , bool _withBall = false , double _max_abs_angle_diff = 0.7, double _lookAhead = 2000.0);
    
    void setTargetPos(const Vec& target) throw(TribotsException);
    bool checkInvocationCondition(const Time&) throw();
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    
    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);
    


  protected:
    Vec    target;
    bool   withBall;
    double max_abs_angle_diff;
    double mindist;
    double lookAhead;
    int    cycles_active;
  };
  
};

#endif
