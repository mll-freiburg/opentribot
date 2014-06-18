#include "BOppStandardSituation.h"
#include "BPreOwnThrowIn.h"
#include "BPreOwnKickOff.h"
#include "../../../WorldModel/WorldModel.h"

namespace Tribots {
	BOppStandardSituation::BOppStandardSituation() throw() : BDIBehavior("BOppStandardSituation") {
		addOption(new BPreOppThrowIn());
	}



	bool BOppStandardSituation::checkInvocationCondition(const Time& time) throw() {
		const BallLocation& ball = MWM.get_ball_location(time);

		if(ball.pos_known == BallLocation::unknown || ball.pos_known == BallLocation::raised)
			return false;

		return BDIBehavior::checkInvocationCondition(time);
	}



	bool BOppStandardSituation::checkCommitmentCondition(const Time& time) throw() {
		const BallLocation& ball = MWM.get_ball_location(time);
		
		if(ball.pos_known == BallLocation::unknown || ball.pos_known == BallLocation::raised)
			return false;

		// give up control after at most 15 seconds of freePlay
		if(MWM.get_game_state().refstate == freePlay) {
			if(time.diff_sec(freePlayTime) > 15)
				return false;
		} else {
			freePlayTime = time;
		}

		return BDIBehavior::checkCommitmentCondition(time);
	}
}
