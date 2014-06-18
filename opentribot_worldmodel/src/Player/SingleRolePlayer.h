
#ifndef Tribots_SingleRolePayer_h
#define Tribots_SingleRolePayer_h

#include "PlayerType.h"


namespace Tribots {

  /** Abstrakte Klasse, die die Methoden get_role (), set_role (const char*), get_list_of_roles () 
      implementiert fuer einen Spielertyp mit genau einer moeglichen Rolle "default" */
  class SingleRolePlayer : public PlayerType {
  public:
    SingleRolePlayer () throw (std::bad_alloc);
    ~SingleRolePlayer () throw () {;}

    /** aktuelle Rolle anfragen */
    const char* get_role () throw ();
    /** Rolle wechseln, falls moeglich; Arg1: Rollenbeschreibung, Arg2: Erfolg? */
    bool set_role (const char*) throw ();
    /** Liste aller Rollen anfragen */
    const std::vector<std::string>& get_list_of_roles () throw ();

    void updateTactics (const TacticsBoard&) throw () {;}

  private:
    std::vector<std::string> roles;
  };

}

#endif

