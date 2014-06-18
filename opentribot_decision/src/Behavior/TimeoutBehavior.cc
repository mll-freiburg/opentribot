
#include "TimeoutBehavior.h"

using namespace Tribots;

TimeoutBehavior::TimeoutBehavior(unsigned int msec, std::string name, bool registerCycleCallback) :
    Behavior (name, registerCycleCallback) {
  duration=msec;
}

void TimeoutBehavior::setTimeout (unsigned int msec) throw () {
  duration=msec;
}

void TimeoutBehavior::restartTimer (Time t) throw () {
  timer=t;
}

TimeoutBehavior::~TimeoutBehavior() throw() {;}

bool TimeoutBehavior::checkCommitmentCondition(const Time& t) throw() {
  return t.diff_msec(timer)<static_cast<int>(duration);
}

void TimeoutBehavior::gainControl(const Time& t) throw(TribotsException) {
  timer=t;
}
