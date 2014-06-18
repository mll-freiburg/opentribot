#ifndef TRIBOTS_BSTANDARDSITUATION_H
#define TRIBOTS_BSTANDARDSITUATION_H

#include "../../Behavior.h"
#include "../../../Fundamental/BallLocationHysteresis.h"

namespace Tribots {
	class BStandardSituation : public Behavior {
		public:
			BStandardSituation(std::string name = "BStandardSituation");

			DriveVector getCmd(const Time& time) throw();

		protected:
			// returns the time in ms since the ball has been released
			inline int getExecutionTime(const Time& time) {
				return executionTime.diff_msec(time);
			}

			inline const BallLocationHysteresis& getArea() { 
				return ballArea; 
			}

			// communicate different phases of a standard situation
			int getExecutionPhase();
			void setExecutionPhase(int n);
			
			// behavior before ball is released, usually positioning of players
			virtual DriveVector getCmdBefore(const Time& time) throw() = 0;

			// behavior after ball has been released, typically execution of standard situation
			virtual DriveVector getCmdAfter(const Time& time) throw() = 0;

		private:
			Time executionTime;
			int executionPhase;
			
			// determines the gross location of the ball
			BallLocationHysteresis ballArea;
	};
}

#endif
