#ifndef _SPBEHAVIOR_H_
#define _SPBEHAVIOR_H_

#include "Behavior.h"
#include <vector>

namespace Tribots {

  /** Class SPBehavior (SP=Sequential Processing) can be used as generic
      class to implement a sequence of behaviors in which one after the other
      is called. Append the behaviors which should become the stages of
      execution in the desired order. For each stage it is possible to specify
      whether this stage might be skipped if its invocation condition is false
      and whether it might be interrupted by a subsequent stage if the
      invocation condition of the subsequent stage is true.

      A stage remains active until its commitment condition becomes false or
      the invocation condition of a subsequent stage becomes true (the latter
      case can be suppressed by disallowing the present stage to be
      interrupted). If a stage has finished execution the next one in the
      list of stages will become active if its invocation condition is true.
      If its invocation condition is false but the SPBehavior is allowed to
      skip this stage, the subsequent stage will become active. If non of the
      remaining stages can become active the commitment condition of the
      SPBehavior itself will become false.

      If the SPBehavior loses control (i.e. itself becomes inactive) it
      restarts the sequence of stages from the begining the next time it
      becomes active again.

      The behaviors which are appended to the list of stages are controled by
      the SPBehavior as soon as they are appended. This means that the
      destructor of the SPBehavior will destroy all the behaviors.

      Example: in a set-play you might want to have a sequence of behaviors like:
      - (1) Go to a waiting position until the referee restarts the game
      - (2) Go to a position close to the ball
      - (3) Approach the ball
      - (4) Kick
      - (5) Go away from the ball

      Stage (1) might be skipped if the referee has already started the game.
      Stage (4) might interrupt stage (3) as soon as the robot touches the
      ball. Stages (2), (3), and (4) must not be skipped. Stage (4) might give
      up after one cycle already (commitmentCondition always true) */
  class SPBehavior : public Behavior {
  public:
    /** Constructor takes as argument the name of the behavior */
    SPBehavior(std::string name="SPBehavior");
    /** destroy all stages */
    virtual ~SPBehavior() throw();

    /** true if the invocation condition of the first stage (one of the first
        stages if the first stage might be skipped) is true */
    virtual bool checkInvocationCondition(const Time&) throw();
    /** true if the commitment condition of the present stage is true
        or the invocation condition of the next stage (one of the subsequent
        stages if stages might be skipped) is true */
    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    virtual void cycleCallBack(const Time&) throw();  ///< resets scheduledStage
    virtual void loseControl(const Time&) throw(TribotsException);  ///< resets presentStage
    virtual void updateTactics (const TacticsBoard&) throw ();
    virtual void printHierarchy (std::ostream&, unsigned int) const throw ();
    virtual void setPlayerRole (const char*) throw ();
    std::string getIntentionName () const throw ();

    /** append a behavior to the set of stages. Arg2 controls whether the
        stage might be skipped (true) or not (false). Arg3 controls whether a
        subsequent behavior is allowed to interrupt the stage (true) or not
        (false). Arg2 and Arg3 override the standard settings of allowSkip and
        allowInterrupt for this stage. Arg4 is the interrupt influence length
        which restricts the permission to interrupt predecessing behaviors up to
        the latest (Arg4) behaviors. Arg4=0 disallows the behavior to interrupt
        others, Arg4=1 allows to interrupt only the stage before, etc.
        Arg4>Number of stages allows the behavior to interrupt all
        predecessing stages. If (Arg5) is true the behavior will be deleted
        in the destructor of SPBehavior. */
    void appendStage (Behavior*, bool, bool, int, bool=true) throw (std::bad_alloc);
    /** append a behavior to the set of stages. Arg2 controls whether the
        stage might be skipped (true) or not (false). Arg3 controls whether a
        subsequent behavior is allowed to interrupt the stage (true) or not
        (false). Arg2 and Arg3 override the standard settings of allowSkip and
        allowInterrupt for this stage. If (Arg4) is true the behavior will be
        deleted in the destructor of SPBehavior. */
    void appendStage (Behavior*, bool =false, bool=false, bool=true) throw (std::bad_alloc);

  private:
    struct Stage {
      Behavior* behavior;
      bool allowSkip;
      bool allowInterrupt;
      int interruptInfluence;
      bool ownBehavior;
    };

    /** List of stages in order of processing. stages[0] is the first stage,
        stages[1] the second one etc. */
    std::vector<Stage> options;

    int presentIntention;  ///< index of present stage (or -1) if inactive
    int scheduledIntention;  ///< index of next stage that can be executed
    /** is set to true by internalSPUpdateIntention if the commitment
        condition of the intention is false */
    bool intentionFinished;

    /** check which is the next stage that should become active
        (internal method) */
    void internalSPUpdateIntention (const Time&) throw();
    bool internalSPCheckConditions (const Time&) throw();
  };

}

#endif
