
#ifndef Tribots_BVolleyBall_h_
#define Tribots_BVolleyBall_h_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysVolley.h"

namespace Tribots {

  /** Nimmt den Ball volley in vollem Lauf. Richtet sich stets auf gegnerisches Tor aus
      Reagiert nur bei einem Querpass und wenn der Roboter bereits auf das gegnerische
      Tor ausgerichtet ist */
  class BVolleyApproach : public Behavior {
  public:
    BVolleyApproach () throw ();
    virtual ~BVolleyApproach() throw() {;}
    virtual DriveVector getCmd(const Time&) throw ();
    virtual bool checkInvocationCondition (const Time&) throw ();
    virtual bool checkCommitmentCondition (const Time&) throw ();
    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void cycleCallBack(const Time&) throw ();

  protected:
    /** geometrischen Teil der Invoc./Com. Condition pruefen; arg2=invocation? */
    virtual bool checkGeometricCondition (const Time&, bool) throw (); 

    SPhysVolley volley_skill;
    double volley_probability;
    bool volley_decision;
  };

}

#endif 
