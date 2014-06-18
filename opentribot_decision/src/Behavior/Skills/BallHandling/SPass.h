#ifndef _SPASS_H_
#define _SPASS_H_

#include "SDribbleBallToPos.h"

namespace Tribots {
  
  class SPass : public Skill {
  public:
    SPass(int kickDuration=30);
    virtual ~SPass() throw ();

    virtual DriveVector	getCmd(const Time&) throw(TribotsException);
    virtual void setParameters(const Vec& target, double transVel, double shootErrorLeft = 2.0, double shootErrorRight = 2.0) 
        throw(TribotsException);
    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);
		virtual void cycleCallBack(const Time& t) throw();

  protected:
		void calcHeadingDifference(const Time& t) throw();
		
		SDribbleBallToPos *skill;
		Vec target;

		double shootErrorLeft;
		double shootErrorRight;
		
		bool emergencyKick;
		double hdiff;
		bool active;
    
    int kickDuration;
	};
}

#endif
