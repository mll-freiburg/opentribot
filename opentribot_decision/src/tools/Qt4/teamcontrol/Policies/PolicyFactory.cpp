
#include "PolicyFactory.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

TribotsTools::PolicyFactory* TribotsTools::PolicyFactory::the_only_factory (NULL);

PolicyFactory::PolicyFactory () throw () {;}

PolicyFactory* PolicyFactory::get_policy_factory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new PolicyFactory;
  return the_only_factory;
}

PolicyFactory::~PolicyFactory() throw () {;}

void PolicyFactory::sign_up (const std::string descriptor, PolicyBuilder* pointer) throw (std::bad_alloc) {
  list_of_plugins [descriptor] = pointer;
}

Policy* PolicyFactory::get_policy (const std::string descriptor, const ConfigReader& reader) throw (TribotsException,bad_alloc,invalid_argument) {
  map<std::string, PolicyBuilder*>::iterator mit = list_of_plugins.find (descriptor);
  Policy* new_wm = NULL;
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_policy (descriptor, reader);
  else
    throw invalid_argument (string("unknown policy type ")+descriptor);

  return new_wm;
}

void PolicyFactory::policy_list (std::vector<std::string>& list) const throw (std::bad_alloc) {
  map<string, PolicyBuilder*>::const_iterator it = list_of_plugins.begin();
  unsigned int sz = list_of_plugins.size();
  list.resize (sz);
  for (unsigned int i=0; i<sz; i++)
    list [i] = (it++)->first;
}
