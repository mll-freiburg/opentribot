#include "MultiRolePlayer.h"

namespace Tribots {

MultiRolePlayer::MultiRolePlayer (const char *roles[], int size) 
  throw (std::bad_alloc)
  : activeRole(0)
{
  if (size == 0 && roles != 0) {   // rollen angegeben, aber keine arraygroesse
    throw std::bad_alloc();
  }
  if (size == 0) {
    this->roles.push_back("default");
  }
  else {
    for (int i=0; i < size; i++) {
      this->roles.push_back(roles[i]);
    }
  }
}

const char* 
MultiRolePlayer::get_role () throw () {
  return roles[activeRole].c_str();
}

bool 
MultiRolePlayer::set_role (const char* s) throw () {
  for (unsigned int i=0; i < roles.size(); i++) {
    if (roles[i] == std::string(s)) {
      activeRole = i;
      return true;
    }
  }
  return false; // have not found that role
}

const std::vector<std::string>& 
MultiRolePlayer::get_list_of_roles () throw () {
  return roles;
}

}
