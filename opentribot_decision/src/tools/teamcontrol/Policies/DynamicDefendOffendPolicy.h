
#ifndef TribotsTools_DynamicDefendOffendPolicy_h
#define TribotsTools_DynamicDefendOffendPolicy_h

#include "StaticPolicy.h"

namespace TribotsTools {

  /** Dynamische Rollenwechselstrategie; Um eine wirkliche Strategie 
      zu bekommen, muss von StaticPolicy abgeleitet werden und Name 
      ausgefuellt werden */
  class DynamicDefendOffendPolicy : public Policy {
  public:
    DynamicDefendOffendPolicy (Policy*, Policy*);   ///< Arg1: Verteidigungsformation, Arg2: Angriffsformation
    ~DynamicDefendOffendPolicy () throw ();         ///< delete auf die beiden Teilformationen
    const char* get_name () const throw ();
    void update () throw ();

  protected:
    std::string policy_name;            ///< Name der Strategie
  private:
    Policy* defensive_policy;
    Policy* offensive_policy;
    bool was_offensive;
  };

}

#endif
