#include "Skill.h"
#include "CycleCallBackRegistry.h"

using namespace std;
namespace Tribots {

  Skill::Skill(string name) 
    : name(name)
  {
    CycleCallBackRegistry::getRegistry()->registerCallBack(this);
  }

  Skill::~Skill () throw () {
    CycleCallBackRegistry::getRegistry()->unregisterCallBack(this);
  }

}
