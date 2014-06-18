
#ifndef TribotsTools_Coach_h
#define TribotsTools_Coach_h

#include "../Policies/Policy.h"
#include "../../../../Fundamental/ConfigReader.h"
#include "../../../../Fundamental/Time.h"
#include "../../../../Structures/TribotsException.h"
#include <vector>


namespace TribotsTools {

  /** Coach, uebernimmt Team-Koordination */
  class Coach {
  public:
    /** Konstruktor; Arg1: Configfile fuer Taktikvorgaben */
    Coach (const Tribots::ConfigReader&) throw (Tribots::TribotsException);
    ~Coach () throw ();

    /** Treffen der taktischen Entscheidungen und Kommunikation mit den Spielern */
    void update () throw ();

  private:
    const Tribots::ConfigReader& cfg;
    Policy* policy;                              // Rollenwechselstrategie
    std::vector<std::string> ignore_broadcast_prefix;  // Liste mit Prefixen, die im Broadcast-Modus nicht weiterkommuniziert werden

    // Teile von update():
    void sl_mirror_hint ();
    void ball_position ();
    void ball_posession ();
    void broadcast ();
    void extra_message ();
    double go_to_ball_ranking(int playernumber);
    int last_ball_owner;
    double owner_bonus;
    
  };

}

#endif
