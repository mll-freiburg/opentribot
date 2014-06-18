
#include "WorldModelFactory.h"

using namespace Tribots;
using namespace std;

Tribots::WorldModelFactory* Tribots::WorldModelFactory::the_only_factory (NULL);

WorldModelFactory::WorldModelFactory () throw () {;}

WorldModelFactory* WorldModelFactory::get_world_model_factory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new WorldModelFactory;
  return the_only_factory;
}

WorldModelFactory::~WorldModelFactory() throw () {;}

void WorldModelFactory::sign_up (const std::string descriptor, WorldModelBuilder* pointer) throw (std::bad_alloc) {
  list_of_plugins [descriptor] = pointer;
}

WorldModelType* WorldModelFactory::get_world_model (const std::string descriptor, const ConfigReader& reader) throw (TribotsException,bad_alloc,invalid_argument) {
  map<std::string, WorldModelBuilder*>::iterator mit = list_of_plugins.find (descriptor);
  WorldModelType* new_wm = NULL;
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_world_model (descriptor, reader, NULL);
  else
    throw invalid_argument (string("unknown world model type ")+descriptor);
  bool b;
  if (reader.get("add_write_world_model", b)>0 && b) {
    mit = list_of_plugins.find ("AddWriteWorldModel");
    if (mit!=list_of_plugins.end())
      new_wm = mit->second->get_world_model (string("AddWriteWorldModel"), reader, new_wm);
    else
      throw invalid_argument ("unknown world model type AddWriteWorldModel");
  }
  return new_wm;
}

  
