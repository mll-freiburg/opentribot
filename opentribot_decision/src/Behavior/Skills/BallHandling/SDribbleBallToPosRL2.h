#ifndef _SDRIBBLEBALLTOPOSRL2_H_
#define _SDRIBBLEBALLTOPOSRL2_H_

#include "SDribbleBallToPosInterface.h"
#include "NFQcontroller.h"

namespace Tribots {
  
  class RLDribbleControllerFixed2;

  /** Dieser Skill dribbelt den Ball zur angegeben Position und macht dabei
   *  ausladende Bewegungen zur Seite, um die Kontrolle ueber den Ball zu 
   *  behalten. Keine Hindernisvermeidung in diesem Skill. Ersatz fuer
   *  klassisches SDribbleBallToPos. */
  class SDribbleBallToPosRL2 : public SDribbleBallToPosInterface {
  public:
    SDribbleBallToPosRL2();
    virtual ~SDribbleBallToPosRL2() throw ();

    virtual DriveVector	getCmd(const Time&)			throw(TribotsException);
    virtual void setParameters(const Vec& target, double transVel, bool) 
        throw(TribotsException);

  protected:
    Vec			  actual_world_target;
    RLDribbleControllerFixed2*  controller;
    unsigned int          odim, adim;
    void get_observation(double *observation) throw(TribotsException);
  };

  /** RL Kontroller fixiert auf den Zustand vom 1. Juni 07. Hier alle
   *  Methoden zum Lernen entfernt. */
  class RLDribbleControllerFixed2 {
  public:
    RLDribbleControllerFixed2(unsigned int _odim, unsigned int _adim);
    virtual ~RLDribbleControllerFixed2();
    virtual bool getCmd(const double* observation, double* action);

    void setGoStraightVelocity(double vel) { spec.gostraight_speed=vel; }
  protected:
    NFQcontroller nfqcontroller;
    unsigned int odim;
    unsigned int adim;
    struct{
      double gostraight_speed;
      double gostraight_threshhold;
      bool training;
    } spec;
  };
}

#endif
