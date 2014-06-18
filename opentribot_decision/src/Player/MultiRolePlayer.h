
#ifndef Tribots_MultiRolePayer_h
#define Tribots_MultiRolePayer_h

#include "PlayerType.h"


namespace Tribots {

  /** 
   * Abstrakte Klasse, die die Methoden get_role (), set_role (const char*), 
   * get_list_of_roles () implementiert fuer einen Spielertyp mit einer Liste
   * von moeglichen Rollen 
   */
  class MultiRolePlayer : public PlayerType {
  public:
    MultiRolePlayer (const char *roles[] = 0, int size = 0) 
      throw (std::bad_alloc);
    ~MultiRolePlayer () throw () {;}

    /** aktuelle Rolle anfragen */
    const char* get_role () throw ();
    /** Rolle wechseln, falls moeglich; Arg1: Rollenbeschreibung, Arg2: Erfolg? */
    bool set_role (const char*) throw ();
    /** Liste aller Rollen anfragen */
    const std::vector<std::string>& get_list_of_roles () throw ();

    void updateTactics (const TacticsBoard&) throw () {;}

  protected:
    std::vector<std::string> roles;
    int activeRole;
  };

}

#endif

