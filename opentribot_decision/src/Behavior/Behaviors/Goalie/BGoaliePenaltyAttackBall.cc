
#include "BGoaliePenaltyAttackBall.h"
#include "../../../WorldModel/WorldModel.h"

using namespace Tribots;


BGoaliePenaltyAttackBall::BGoaliePenaltyAttackBall (bool obs, Vec p1, Vec p2, bool kck) throw () : BGoalieAttackBall (obs, p1, p2, kck) {
  name="BGoaliePenaltyAttackBall";
}

bool BGoaliePenaltyAttackBall::checkInvocationCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==opponentPenalty) && BGoalieAttackBall::checkInvocationCondition (t);
}

bool BGoaliePenaltyAttackBall::checkCommitmentCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==opponentPenalty) && BGoalieAttackBall::checkCommitmentCondition (t);
}
