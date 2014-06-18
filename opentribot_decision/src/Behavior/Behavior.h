#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "Skill.h"
#include <iostream>

namespace Tribots {

  /** Reactive or deliberative behavior. Behaviors are more complex than skills
   *  and do not need any parameters after their construction. They collect all
   *  information necessary to generate a reasonable command from the world 
   *  model. They have explicit invocation and commitment conditions. Behaviors
   *  may call several different skills (or even behaviors) to realize a 
   *  complex policy.
   *
   *  Example:
   *
   *  "Dribble to position x, y"
   *  - invocation condition  (IC): owns ball && velocity < 1 m/s
   *  - commitment condition  (CC): owns ball
   *
   */
  class Behavior : public Skill {
  public:
    /** 
     * true, if in this state of the world model this behavior would return a
     * reasonable action
     * \default true
     * \param Time the expected execution time of the present command cycle
     * \returns true if the commitment condition is fullfilled. default
     *          implementation returns always true.
     */
    virtual bool checkCommitmentCondition(const Time&) throw()
    { return true; }

    /**
      * true, if it is possible to start this action in the current state
      * of the worldmedel. generally implies ckeckCommitmentCondition==true.
      * \default true
      */
    virtual bool checkInvocationCondition(const Time&) throw()
    { return true; }

    /** returns the (estimated) probability to reach the behavior's target 
        state wihtin "a short time period" */
    virtual double getSuccessProbability(const Time&) const throw() { return 1.; };

    /** notify a player role change to the behavior */
    virtual void setPlayerRole (const char*) throw () {;}

    /** die Verhaltenshierarchie fuer debug in einen Stream schreiben;
       Arg1: Ausgabestream,
       Arg2: Hierarchiestufe (0=Wurzelknoten) */
    virtual void printHierarchy (std::ostream&, unsigned int) const throw ();

    /** Die Intention zurueckliefern, ggf. durch die Hierarchie hindurch */
    virtual std::string getIntentionName () const throw () { return name; }

  protected:
    Behavior(std::string name="Behavior") : Skill(name) {}
  };
}

#endif
