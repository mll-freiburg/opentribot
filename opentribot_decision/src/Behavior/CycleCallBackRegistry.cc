#include "CycleCallBackRegistry.h"
#include "Skill.h"

namespace Tribots
{

using namespace std;

CycleCallBackRegistry* CycleCallBackRegistry::theRegistry = 0;

CycleCallBackRegistry::CycleCallBackRegistry()
{}

CycleCallBackRegistry::~CycleCallBackRegistry()
{
  if (theRegistry == this) {
    theRegistry = 0;
  }
}

CycleCallBackRegistry* 
CycleCallBackRegistry::getRegistry()
{
  if (theRegistry == 0) {
    theRegistry = new CycleCallBackRegistry();
  }
  return theRegistry;
}

void
CycleCallBackRegistry::registerCallBack(Skill* skill)
{
  registeredSkills.push_back(skill);
}

void
CycleCallBackRegistry::unregisterCallBack(Skill* skill)
{
  for (unsigned int i=0; i<registeredSkills.size(); i++) {
    if (registeredSkills[i]==skill)  // Zeigervergleich
      registeredSkills[i]=NULL;
  }
}

void
CycleCallBackRegistry::callCycleCallBacks(const Time& time)
{
  for (unsigned int i=0; i < registeredSkills.size(); i++) {
    if (registeredSkills[i])
      registeredSkills[i]->cycleCallBack(time);
  }
}

};
