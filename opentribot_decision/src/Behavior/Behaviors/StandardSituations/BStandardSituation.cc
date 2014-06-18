#include <string>
#include <sstream>
#include "BStandardSituation.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"

using std::string;
using std::stringstream;

namespace Tribots {
	BStandardSituation::BStandardSituation(string name) : executionPhase(0) {
	}



	DriveVector BStandardSituation::getCmd(const Time& time) throw() {
		if(MWM.get_game_state().refstate == freePlay) {
			// ball has been released
			return getCmdAfter(time);
		} else {
			executionTime = time;
			ballArea.update(time);
			return getCmdBefore(time);
		 }
	}



	int BStandardSituation::getExecutionPhase() {
		string str = MWM.get_message_board().scan_for_prefix("standardsituation_phase");

		if(str.length() > 0) {
			stringstream strstr(str);

			strstr >> str;
			strstr >> executionPhase;

			std::cout << "IN: standardsituation_phase: " << executionPhase;
			MWM.get_message_board().clear();
		}

		return executionPhase;
	}



	void BStandardSituation::setExecutionPhase(int n) {
		executionPhase = n;

		// inform other players
		stringstream strstr;

		strstr << "standardsituation_phase: " << n;
		MWM.get_message_board().publish(strstr.str());

		std::cout << "OUT: standardsituation_phase: " << n;
	}
}
