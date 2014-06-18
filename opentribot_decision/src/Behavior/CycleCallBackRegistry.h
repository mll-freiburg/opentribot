#ifndef _CYCLECALLBACKREGISTRY_H_
#define _CYCLECALLBACKREGISTRY_H_

#include <vector>
#include "../Fundamental/Time.h"

namespace Tribots
{

  class Skill;

  class CycleCallBackRegistry
  {
  public:
    static CycleCallBackRegistry* getRegistry();
	  virtual ~CycleCallBackRegistry();
    
    /** register a skill in order to call its cycleCallBack method      */
    void registerCallBack(Skill* skill);
    /** inverse operation to registerCallBack, remove skill from list of registered skills      */
    void unregisterCallBack(Skill* skill);
    /** call the cycleCallback method of every registered skill once */
    void callCycleCallBacks(const Time& time);
  protected:
    static CycleCallBackRegistry* theRegistry;
    std::vector<Skill*> registeredSkills;
    CycleCallBackRegistry();
  };

};

#endif //_CYCLECALLBACKREGISTRY_H_
