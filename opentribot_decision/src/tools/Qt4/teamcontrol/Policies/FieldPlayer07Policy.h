#ifndef _TribotsTools_FieldPlayer07Policy_h_
#define _TribotsTools_FIeldPlayer07Policy_h_

#include "StaticPolicy.h"

namespace TribotsTools {

class FieldPlayer07Policy : public StaticPolicy {
public:
  FieldPlayer07Policy();
  ~FieldPlayer07Policy() throw() {;}
  
  void update () throw ();

protected:
  int dynamicChangeCycleCounter;
  
};

}

#endif
