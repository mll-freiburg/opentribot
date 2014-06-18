
#include "VisionFactory.h"

using namespace Tribots;
using namespace std;

Tribots::VisionFactory* Tribots::VisionFactory::the_only_factory (NULL);

VisionFactory::VisionFactory () throw () {;}

VisionFactory* VisionFactory::get_vision_factory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new VisionFactory;
  return the_only_factory;
}

VisionFactory::~VisionFactory() throw () {;}

void VisionFactory::sign_up (const std::string descriptor, VisionBuilder* pointer) throw (std::bad_alloc) {
  list_of_plugins [descriptor] = pointer;
}

VisionType* VisionFactory::get_vision (const std::string descriptor, std::string section, const ConfigReader& reader) throw (TribotsException,bad_alloc,invalid_argument) {
  map<std::string, VisionBuilder*>::iterator mit = list_of_plugins.find (descriptor);
  VisionType* new_wm = NULL;
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_vision (descriptor, section, reader, NULL);
  else
    throw invalid_argument (string("unknown vision ")+descriptor);

  return new_wm;
}

  
