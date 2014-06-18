
#ifndef TribotsTools_RefboxControl_h
#define TribotsTools_RefboxControl_h

#include "../../../../Fundamental/ConfigReader.h"
#include "../../../../Structures/TribotsException.h"
#include "../../../../WorldModel/Orga/RefereeStateMachine.h"
#include "RefboxClient.h"


namespace TribotsTools {

  /** RefboxControl, uebernimmt die Verwaltung der Refereebox-Signale */
  class RefboxControl {
  public:
    /** Konstruktor; Arg1: Configfile */
    RefboxControl (const Tribots::ConfigReader&) throw (Tribots::TribotsException);
    ~RefboxControl () throw ();

    /** Kommunikation mit Refereebox und Aktualisierung der Refereestates */
    void update () throw ();
    /** Synchronisationssignale */
    void update_synch () throw ();

  private:
    int own_half;
    int team_color;
    bool was_connected_to_refbox;
    RefboxClient refbox;
    Tribots::RefereeStateMachine* automaton;
    Tribots::RefereeStateMachine* inverse_automaton;
    unsigned short int synch_signal;     // Synchronisationssignal
    Tribots::Time synch_signal_time; // Zeitstempel fuer naechstes Synchronisationssignal
  };

}

#endif
