#ifndef _TribotsTools_FieldPlayerPolicy_h_
#define _TribotsTools_FIeldPlayerPolicy_h_

#include "StaticPolicy.h"

namespace TribotsTools {

class FieldPlayerPolicy : public StaticPolicy {
public:
  FieldPlayerPolicy();
  ~FieldPlayerPolicy() throw() {;}
  
  void update () throw ();

protected:
  int dynamicChangeCycleCounter;
  
};

}

#endif
