#ifndef _TIMEOUTBEHAVIOR_H_
#define _TIMEOUTBEHAVIOR_H_

#include "Behavior.h"
#include "../Fundamental/Time.h"

namespace Tribots {

  /** Class TimeoutBehavior can be used to generate behaviors that should
      become inactive after a maximal period of time. The class contains a
      timer that is started whenever the behavior becomes active and that
      automatically sets the commitment condition to false after a specified
      amount of milliseconds. */
  class TimeoutBehavior : public Behavior {
  protected:
    /** reimplement the constructor for your own purpose. "msec" is the
        maximal amount of time in milliseconds after which the behavior should
        become inactive */
    TimeoutBehavior(unsigned int msec, std::string name="TimeoutBehavior", bool registerCycleCallback=false);
    /** set the timeout in milliseconds */
    void setTimeout (unsigned int) throw ();
    /** restart the timer using the given timestamp */
    void restartTimer (Time) throw ();
  public:
    virtual ~TimeoutBehavior() throw();
    /** false if the time passed since activation is longer than "duration" */
    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual void gainControl(const Time&) throw(TribotsException);
  private:
    Time timer;
    unsigned int duration;
  };
}

#endif
