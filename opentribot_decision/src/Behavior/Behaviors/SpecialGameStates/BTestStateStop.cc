
#include "BTestStateStop.h"
#include "../../../WorldModel/WorldModel.h"

using namespace std;
using namespace Tribots;

BTestStateStop::BTestStateStop () throw () {
  name="BTestStateStop";
}

bool BTestStateStop::checkInvocationCondition(const Time&) throw() {
  return MWM.get_game_state().refstate == testState1
      || MWM.get_game_state().refstate == testState2
      || MWM.get_game_state().refstate == testState3
      || MWM.get_game_state().refstate == testState4
      || MWM.get_game_state().refstate == testState5
      || MWM.get_game_state().refstate == testState6
      || MWM.get_game_state().refstate == testState7
      || MWM.get_game_state().refstate == testState8;
}

bool BTestStateStop::checkCommitmentCondition(const Time& t) throw() {
  return checkInvocationCondition (t);
}
