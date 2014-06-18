#ifndef _TRIBOTS_BPREOWNSTANDARDSITUATIONSTACK_H_
#define _TRIBOTS_BPREOWNSTANDARDSITUATIONSTACK_H_

#include "../../BDIBehavior.h"

namespace Tribots {
	class BOwnStandardSituation : public BDIBehavior {
		public:
			BOwnStandardSituation() throw();

			bool checkInvocationCondition(const Time& time) throw();
			bool checkCommitmentCondition(const Time& time) throw();

		private:
			// last point in time before release of the ball
			Time freePlayTime;
	};
}

#endif
