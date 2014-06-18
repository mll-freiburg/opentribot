// 
// File:   BSecurity.h
// Author: stephangabler
//
// Created on 14. April 2008, 16:11
//

#ifndef _BSECURITY_H
#define	_BSECURITY_H

#include "../../../WorldModel/WorldModel.h"
#include "../BasicMovements/BEmergencyStop.h"

namespace Tribots {

  /** ein Verhalten, das wahrend der Testzustaende bei stuck den Roboter anhaelt.
	 *	ausserdem stoppt es wenn ein HInderniss in einem Abstand von weniger als einem
	 *  Meter vor ihm auftaucht und es so breit ist, dass er nicht vorbei kommt
	 **/
  class BSecurity : public BEmergencyStop {
		public:
			BSecurity ();
			~BSecurity () throw ();
			bool checkInvocationCondition(const Time& t) throw();
			bool checkCommitmentCondition(const Time& t) throw();
		protected:
			double robsize;

  };
	

}



#endif	/* _BSECURITY_H */

