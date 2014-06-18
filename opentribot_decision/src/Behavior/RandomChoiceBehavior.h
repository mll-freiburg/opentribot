#ifndef _RANDOMCHOICEBEHAVIOR_H_
#define _RANDOMCHOICEBEHAVIOR_H_

#include "Behavior.h"
#include <vector>
#include <iostream>

namespace Tribots {

  /** RandomChoiceBehavior is a generic Behavior that can be used to implement
      a random choice between several behaviors. All possible behaviors are
      added to the RandomChoiceBehavior using the addOption() method.
      The behavior selects one option by random according to the
      probabilities. The selected option is called intention. If the sum of
      probabilities is unequal to 1 the RandomChoiceBehavior internally
      rescales the probabilities.

      The RandomChoiceBehavior makes a new random decision whenever it gains
      control. When it loses control it unsets this decision. Hence, the
      invocation condition of the RandomChoiceBehavior is equal to the
      OR-connection of the invocation conditions of all options. The random
      choice is taken only from those options which have true invocation
      condition at the point of gaining control.

      The commitment condition and the getCmd() method of the
      RandomChoiceBehavior are identical to those of the intention.
      If no option exists the conditions are always false and
      getCmd() will return a zero drive vector.

      Example: a set-play might be executed in different ways, e.g.
      - Dribbling the ball
      - Playing a short pass
      - Playing a long pass

      It might be convenient to implement these alternatives as behaviors and
      select one of them randomly using fixed probabilities and dynamic mode.
      With its invocation condition each option might also control when it
      cannot be used, e.g. playing a long pass might be undesired when the ball
      is close to the own goal so that in such a situation the decision is made
      between the remaining options 'short pass' and 'dribbling'. */
  class RandomChoiceBehavior : public Behavior {
  public:
    /** Constructor takes as argument the name of the behavior */
    RandomChoiceBehavior(std::string name="RandomChoiceBehavior");
    virtual ~RandomChoiceBehavior() throw();
    /** add an option (Arg1) with probability (Arg2). If (Arg3) is true
        the option added will be deleted in the destructor of the
        RandomChoiceBehavior. The options will be added one after the other */
    void addOption (Behavior*, double, bool =true) throw ();

    /** alters the probability of option (Arg1) to the value of (Arg2) */
    void changeProbability (unsigned int, double) throw ();

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual DriveVector getCmd(const Time& time) throw(TribotsException);
    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);

    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void printHierarchy (std::ostream&, unsigned int) const throw ();
    virtual void setPlayerRole (const char*) throw ();
    std::string getIntentionName () const throw ();

  private:
    struct RandomOption {
      Behavior* behavior;
      double probability;  ///< die gesetzte Auswahlwahrscheinlichkeit
      double cprobability;  ///< interne Variable wird bei der Zufallsauswahl verwendet
      bool ownBehavior;  ///< das Behavior am Ende loeschen?
    };
    int intention;  ///< ausgewaehlte Option. -1 bedeutet, derzeit keine Option ausgewaehlt
    std::vector<RandomOption> options;
    /** implements a random selection called by gainControl().
        If blind is true, the choice will be made among
        all options without considering their invocation condition. In
        contrast, if blind is false, only options with true invocation
        conditions are considered. */
    void internalRCRandomChoice (const Time&, bool blind) throw ();

  };

}

#endif
