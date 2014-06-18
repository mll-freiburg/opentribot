#ifndef _BSHOOTEMERGENCY_H_
#define _BSHOOTEMERGENCY_H_

#include "BShootImmediately.h"

namespace Tribots
{

class BShootEmergency : public Tribots::BShootImmediately
{
public:
	BShootEmergency(int hackKickLength=30);
	virtual ~BShootEmergency() throw();

  virtual bool checkCommitmentCondition(const Time&) throw();
  virtual bool checkInvocationCondition(const Time&) throw();  
  
};

};

#endif //_BSHOOTEMERGENCY_H_
