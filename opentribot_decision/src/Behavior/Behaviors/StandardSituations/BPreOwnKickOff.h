#ifndef _TRIBOTS_BPREOWNKICKOFF_H
#define _TRIBOTS_BPREOWNKICKOFF_H

#include "../../Behavior.h"
#include "BStandardSituation.h"

namespace Tribots {
	class BPreOwnKickOff : public BStandardSituation {
		public:
			BPreOwnKickOff() throw();

			virtual bool checkCommitmentCondition(const Time& time) throw();
			virtual bool checkInvocationCondition(const Time& time) throw();

			virtual void gainControl(const Time& time) throw();

		protected:
			// true if kick-off has been executed
			bool executionFinished;

			// remember last position of ball and robot before execution
			Vec vBallPosMem;
			Vec vRobotPosMem;

			// position robots and execute kick-off
			virtual DriveVector getCmdBefore(const Time& time) throw();
			virtual DriveVector getCmdAfter(const Time& time) throw();
	};
}

#endif
