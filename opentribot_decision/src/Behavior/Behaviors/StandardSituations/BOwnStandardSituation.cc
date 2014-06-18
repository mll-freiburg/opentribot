#include "BOwnStandardSituation.h"
#include "BPreOwnThrowIn.h"
#include "BPreOwnKickOff.h"
#include "../../../WorldModel/WorldModel.h"

namespace Tribots {
	BOwnStandardSituation::BOwnStandardSituation() throw() : BDIBehavior("BOwnStandardSituation") {
		addOption(new BPreOwnThrowIn());
		addOption(new BPreOwnKickOff());
	}



	bool BOwnStandardSituation::checkInvocationCondition(const Time& time) throw() {
		const BallLocation& ball = MWM.get_ball_location(time);

		if(ball.pos_known == BallLocation::unknown || ball.pos_known == BallLocation::raised)
			return false;

		return BDIBehavior::checkInvocationCondition(time);
	}



	bool BOwnStandardSituation::checkCommitmentCondition(const Time& time) throw() {
		const BallLocation& ball = MWM.get_ball_location(time);
		
		if(ball.pos_known == BallLocation::unknown || ball.pos_known == BallLocation::raised)
			return false;

		// give up control after at most 5 seconds of freePlay
		if(MWM.get_game_state().refstate == freePlay) {
			if(time.diff_sec(freePlayTime) > 5)
				return false;
		} else {
			freePlayTime = time;
		}

		return BDIBehavior::checkCommitmentCondition(time);
	}
}
