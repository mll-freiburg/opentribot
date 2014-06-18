#ifndef _TRIBOTS_BPREOWNTHROWIN_H_
#define _TRIBOTS_BPREOWNTHROWIN_H_

#include "../../Behavior.h"
#include "BStandardSituation.h"

namespace Tribots {
	class BPreOwnThrowIn : public BStandardSituation {
		public:
			BPreOwnThrowIn() throw();

			virtual bool checkCommitmentCondition(const Time& time) throw();
			virtual bool checkInvocationCondition(const Time& time) throw();

			virtual void gainControl(const Time& time) throw();

		protected:
			// true if throw-in has been executed
			bool executionFinished;

			// remember position of ball and robot before execution
			Vec vBallPosMem;
			Vec vRobotPosMem;

			// positioning of players and execution of throw-in
			DriveVector getCmdBefore(const Time& time) throw();
			DriveVector getCmdAfter(const Time& time) throw();
	};
}

#endif
