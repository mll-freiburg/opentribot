
#include "SingleRolePlayer.h"

Tribots::SingleRolePlayer::SingleRolePlayer () throw (std::bad_alloc) : roles (1) {
  roles[0]="default";
}

const char* Tribots::SingleRolePlayer::get_role () throw () {
  return roles[0].c_str();
}

bool Tribots::SingleRolePlayer::set_role (const char* s) throw () {
  return (roles[0]==std::string(s));
}

const std::vector<std::string>& Tribots::SingleRolePlayer::get_list_of_roles () throw () {
  return roles;
}
