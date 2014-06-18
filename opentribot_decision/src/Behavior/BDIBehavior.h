#ifndef _BDIBEHAVIOR_H_
#define _BDIBEHAVIOR_H_

#include "Behavior.h"
#include <vector>
#include <iostream>

namespace Tribots {

  /** Generic behavior class that can be used to combine several behaviors.
      The behavior contains a priority list of behaviors (options) that can
      be executed. The option that is executed at present is called intention.

      Everytime the BDIBehavior is called it checks the commitment condition
      of the intention and the invocation conditions of all options. The
      BDIBehavior changes its intention either if the commitment condition of
      the intention is false or if the invocation condition of an option is
      true that has higher priority.

      The BDIBehavior does not specify an order of execution of the options
      but only a priority of execution. An option with higher priority might
      interrupt an option with lower priority even if the lower priority
      option did not finish its movement.

      Example: a goalie might have three basic behaviors which can be
      assembled in a BDIBehavior:
      - (1) Kick away the ball (if the ball is close to it)
      - (2) Protect the goal (if it knows the ball position)
      - (3) Wait in the middle of the goal

      Obviously, option (1) should be executed whenever it is possible
      (highest priority) and option (3) should only be executed when no other
      option is applicable.

      There are two ways to modify the mechanism of a BDIBehavior.
      Firstly, an option can be declared non-interruptable, i.e. this option
      cannot be interrupted by an option with higher priority. This mechanism
      can be used to implement decisions which should not be altered all the
      time. As an example, an attacker might be faced with a situation in which
      it can try to bypass a defender on the right or left side. Once, the
      player has decided to bypass on one side it should stick to this decision
      for a while and only alter its decision when it turns out that bypassing on
      this side cannot be successful, i.e. when the commitment condition of
      this way of bypassing turns to false. Meanwhile, the option that bypasses
      on the other side should not interrupt the intention even though it
      might have higher priority. Only after the intention becomes inactive
      the player might decide to try again on the other side.

      Secondly, an option can be declared sticky which is a mechanism that is
      even stronger than non-interruptability. As soon as a sticky option
      becomes the intention of the BDIBehavior it will not be interrupted by
      any other option and, when its commitment condition becomes false
      the commitment condition of the BDIBehavior as well becomes false.
      The BDIBehavior is allowed to alter its decision only after becoming
      inactive itself. The sticky mechanism can be used to implement
      decisions between different options without the possibility of a second
      attempt. As example, in a set play situation a player might decide to
      chose one option among playing a pass or dribbling the ball. After
      making its decision its intention should not be interrupted or altered
      even if its intention fails. */
  class BDIBehavior : public Behavior {
  public:
    /** Constructor takes as argument the name of the behavior */
    BDIBehavior(std::string name="BDIBehavior");
    virtual ~BDIBehavior() throw();

    /** execute the intention. In case of a change of intention send
        loseControl() and gainControl() to the old and new intention. Yields
        zero vector if no option can become active (undesired case, schould
        be avoided). Postcondition: scheduledIntention=-1 */
    virtual DriveVector getCmd(const Time& time) throw(TribotsException);
    /** true if the invocation condition of at least one option is true */
    virtual bool checkInvocationCondition(const Time& time) throw();
    /** true if commitmentCondition of present intention is true or
        invocationCondition of another option is true */
    virtual bool checkCommitmentCondition(const Time& time) throw();
    virtual void cycleCallBack(const Time&) throw();  ///< resets scheduledIntention
    virtual void loseControl(const Time&) throw(TribotsException);  ///< resets presentIntention

    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void printHierarchy (std::ostream&, unsigned int) const throw ();
    virtual void setPlayerRole (const char*) throw ();
    std::string getIntentionName () const throw ();

    /** Add an option to the end of the priority list. If interrupt is true
        this option might be interruptd by an option with higher priority.
        The new option has lower priority than all other options already
        added. If own is true the behavior will be destroyed in the destructor
        of the BDIBehavior. If sticky is true and once the BDIBehavior has made
        the respective option to its intention it will never change to another
        option until it loses control. */
    void addOption (Behavior*, bool interrupt=true, bool sticky = false, bool own=true) throw ();
  private:
    struct BDIOption {
      Behavior* behavior;
      bool allowInterrupt;
      bool sticky;
      bool ownBehavior;
    };
    std::vector<BDIOption> options; ///< higher order behaviors come first

    /** index of present intention. If presentIntention<0 or
        presentIntention>=options.size() there is no option active */
    int presentIntention;
    /** index of option that was determined to be active. If
        scheduledIntention<0 it has not yet been determined, if
        scheduledIntention>=options.size() there is no intention that is
        willing to become (stay) active. */
    int scheduledIntention;
    /** is set to true by internalBDIUpdateIntention if the commitment
        condition of the intention is false */
    bool intentionFinished;
    /** recalculate scheduledIntention.
        Postcondition: scheduledIntention>=0 */
    void internalBDIUpdateIntention (const Time&) throw ();
    bool internalBDICheckConditions (const Time&) throw ();
  };

}

#endif
