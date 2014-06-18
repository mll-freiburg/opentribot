
#ifndef TribotsTools_StaticPolicy_h
#define TribotsTools_StaticPolicy_h

#include "Policy.h"
#include "../../../../Fundamental/Table.h"
#include "../../../../Fundamental/Time.h"
#include <string>
#include <vector>

namespace TribotsTools {

  /** Statische Rollenwechselstrategie; Um eine wirkliche Strategie 
      zu bekommen, muss von StaticPolicy abgeleitet werden und Name, 
      Spielertyp und die Liste der Rollen ausgefuellt werden */
  class StaticPolicy : public Policy {
  public:
    StaticPolicy (unsigned int n);      ///< Arg1: Maximalzahl Spieler
    ~StaticPolicy () throw () {;}
    const char* get_name () const throw ();
    void update () throw ();
    void observe () throw ();

  protected:
    std::string policy_name;            ///< Name der Strategie
    std::string playertype;             ///< Spielertyp, fuer den diese Stratgie arbeitet
    const unsigned int max_num_players; ///< Maximalzahl derartiger aktiver Spieler
    Tribots::Table<std::string> roles;  ///< Liste mit Rollen: roles[i][j] ist die Rolle des j+1-ten Spielers von hinten bei insgesamt i+1 Spielern vom Typ "playertype"
  private:
    std::vector<Tribots::Time> robot_activation_time;  // wann wurde welcher Roboter altiviert?
  };

}

#endif
