
#include "RCPlayer05Policies.h"
#include "PolicyFactory.h"

using namespace TribotsTools;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class DDABuilder : public TribotsTools::PolicyBuilder {
    static DDABuilder the_builder;
  public:
    DDABuilder (){
      TribotsTools::PolicyFactory::get_policy_factory ()->sign_up (std::string("RC05DDA"), this);
    }
    TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
      return new RCPlayer05::RCPlayerDDAPolicy(reader);
    }
  };
  DDABuilder the_dda_builder;

  class DSABuilder : public TribotsTools::PolicyBuilder {
    static DSABuilder the_builder;
  public:
    DSABuilder () {
      TribotsTools::PolicyFactory::get_policy_factory ()->sign_up (std::string("RC05DSA"), this);
    }
    TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
      return new RCPlayer05::RCPlayerDSAPolicy(reader);
    }
  };
  DSABuilder the_dsa_builder;

  class DDSABuilder : public TribotsTools::PolicyBuilder {
    static DDSABuilder the_builder;
  public:
    DDSABuilder () {
      TribotsTools::PolicyFactory::get_policy_factory ()->sign_up (std::string("RC05DDSA"), this);
    }
    TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
      return new RCPlayer05::RCPlayerDDSAPolicy(reader);
    }
  };
  DDSABuilder the_ddsa_builder;
}





RCPlayer05::RCPlayerDDAPolicy::RCPlayerDDAPolicy (const Tribots::ConfigReader& reader) : StaticPolicy (4, reader) {
  policy_name = "RC05DDA";
  playertype = "RCPlayer05";
  roles(0,0) = roles (1,1) = roles (2,2) = roles (3,2) = roles (3,3) = "Attack2";
  roles (1,0) = "Defend3";
  roles (2,0) = roles (3,0) = "Defend1";
  roles (2,1) = roles (3,1) = "Defend2";
}


RCPlayer05::RCPlayerDSAPolicy::RCPlayerDSAPolicy (const Tribots::ConfigReader& reader) : StaticPolicy (4, reader) {
  policy_name = "RC05DSA";
  playertype = "RCPlayer05";
  roles(0,0) = roles (1,1) = roles (2,2) = roles (3,3) = "Attack2";
  roles (1,0) = roles (2,0) = "Defend3";
  roles (3,0) = "Defend1";
  roles (3,1) = "Defend2";
  roles (2,1) = roles (3,2) = "Support";
}


RCPlayer05::RCPlayerDDSAPolicy::RCPlayerDDSAPolicy (const Tribots::ConfigReader& reader) 
  : DynamicDefendOffendPolicy (new RCPlayer05::RCPlayerDDAPolicy(reader), new RCPlayer05::RCPlayerDSAPolicy(reader)) {
  policy_name = "RC05DDSA";
}
