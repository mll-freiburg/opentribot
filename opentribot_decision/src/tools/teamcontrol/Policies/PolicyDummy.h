
#ifndef TribotsTools_PolicyDummy_h
#define TribotsTools_PolicyDummy_h

#include "Policy.h"

namespace TribotsTools {

  /** eine Policy, die nichts tut */
  class PolicyDummy : public Policy {
  public:
    PolicyDummy () throw ();
    ~PolicyDummy () throw () {;}
    const char* get_name () const throw ();
    void update () throw ();
  private:
    char name [4];
  };

}

#endif
