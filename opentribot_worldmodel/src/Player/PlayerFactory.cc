
#include "PlayerFactory.h"

using namespace Tribots;
using namespace std;

Tribots::PlayerFactory* Tribots::PlayerFactory::the_only_factory (NULL);

PlayerFactory::PlayerFactory () throw () {;}

PlayerFactory* PlayerFactory::get_player_factory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new PlayerFactory;
  return the_only_factory;
}

PlayerFactory::~PlayerFactory() throw () {;}

void PlayerFactory::sign_up (const std::string descriptor, PlayerBuilder* pointer) throw (std::bad_alloc) {
  list_of_plugins [descriptor] = pointer;
}

PlayerType* PlayerFactory::get_player (const std::string descriptor, const ConfigReader& reader) throw (TribotsException,bad_alloc,invalid_argument) {
  map<std::string, PlayerBuilder*>::iterator mit = list_of_plugins.find (descriptor);
  PlayerType* new_wm = NULL;
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_player (descriptor, reader, NULL);
  else
    throw invalid_argument (string("unknown player type ")+descriptor);
  bool b;
  if (reader.get("add_joystick_player", b)>0 && b) {
    mit = list_of_plugins.find ("AddJoystickPlayer");
    if (mit!=list_of_plugins.end())
      new_wm = mit->second->get_player (string("AddJoystickPlayer"), reader, new_wm);
    else
      throw invalid_argument ("unknown world model type AddJoystickPlayer");
  }
  mit = list_of_plugins.find ("AddGotoPosPlayer");
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_player (string("AddGotoPosPlayer"), reader, new_wm);
  else
    throw invalid_argument ("unknown world model type AddGotoPosPlayer");

  return new_wm;
}

void PlayerFactory::player_list (std::vector<std::string>& list) const throw (std::bad_alloc) {
  map<string, PlayerBuilder*>::const_iterator it = list_of_plugins.begin();
  unsigned int sz = list_of_plugins.size();
  list.resize (sz);
  for (unsigned int i=0; i<sz; i++)
    list [i] = (it++)->first;
}
