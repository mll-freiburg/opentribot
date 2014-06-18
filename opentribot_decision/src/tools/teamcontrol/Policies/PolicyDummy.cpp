
#include "PolicyDummy.h"
#include "PolicyFactory.h"

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public TribotsTools::PolicyBuilder {
    static Builder the_builder;
  public:
    Builder () {
      TribotsTools::PolicyFactory::get_policy_factory ()->sign_up (std::string("---"), this);
    }
    TribotsTools::Policy* get_policy (const std::string&, const Tribots::ConfigReader& reader) throw (Tribots::TribotsException,std::bad_alloc) {
      return new TribotsTools::PolicyDummy;
    }
  };
  Builder the_builder;
}




TribotsTools::PolicyDummy::PolicyDummy () throw () {
  name[0]=name[1]=name[2]='-';
  name[3]='\0';
}

const char* TribotsTools::PolicyDummy::get_name () const throw () {
  return name;
}

void TribotsTools::PolicyDummy::update () throw () {;}
