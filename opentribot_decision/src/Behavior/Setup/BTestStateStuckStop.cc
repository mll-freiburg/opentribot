
#include "BTestStateStuckStop.h"

using namespace Tribots;

BTestStateStuckStop::BTestStateStuckStop () {
  name="BTestStateStuckStop";
}

BTestStateStuckStop::~BTestStateStuckStop () throw () {;}

bool BTestStateStuckStop::checkInvocationCondition(const Time& t) throw() {
  RefereeState state = MWM.get_game_state().refstate;
  bool isStuck = MWM.get_robot_location(t).stuck();
  bool isTestState = (state==testState1) 
                  || (state==testState2)
                  || (state==testState3)
                  || (state==testState4)
                  || (state==testState5)
                  || (state==testState6)
                  || (state==testState7)
                  || (state==testState8);
  return isStuck && isTestState;
}

bool BTestStateStuckStop::checkCommitmentCondition(const Time& t) throw() {
  return checkInvocationCondition (t);
}
