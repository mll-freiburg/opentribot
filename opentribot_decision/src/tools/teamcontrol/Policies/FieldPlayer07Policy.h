#ifndef _TribotsTools_FieldPlayer07Policy_h_
#define _TribotsTools_FIeldPlayer07Policy_h_

#include "StaticPolicy.h"
#include "../../../Fundamental/ConfigReader.h"
#include <vector>
#include "../../../Fundamental/Time.h"

namespace TribotsTools {

class FieldPlayer07Policy : public StaticPolicy {
public:
  FieldPlayer07Policy(const Tribots::ConfigReader&);
  ~FieldPlayer07Policy() throw() {;}
  
  void update () throw ();

protected:
  int dynamicChangeCycleCounter;
  std::vector<int> passMessageCounter;
  
  Tribots::Time passReceived;
  int lastReceiverID;
  int lastSenderID;
};

}

#endif
