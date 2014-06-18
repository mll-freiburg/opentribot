
#ifndef _TribotsTools_BehaviorStatistics_h_
#define _TribotsTools_BehaviorStatistics_h_

#include <iostream>
#include <string>
#include <vector>
#include "../../Structures/GameState.h"
#include "../../Fundamental/binary_encoding.h"

namespace TribotsTools {

  /** Klasse, um aus den Logfiles die Intentionen zu lesen und Statistik zu betreiben */
  class BehaviorStatistics {
  private:
    std::vector<std::string> intentions;
    std::vector<std::string> unstripped_intentions;
    std::vector<Tribots::RefereeState> refstates;
    std::vector<Tribots::RefereeState> next_standard;
    std::vector<unsigned int> time_next_standard;

    bool read_textlog (const std::string&) throw ();
    bool read_gslog (const std::string&) throw ();
    void calculate_next_standard () throw (std::bad_alloc);
  public:
    /** Logfile lesen; Arg1: Name des Logfile; liefert true, falls Logfile gelesen werrden konnte */
    bool read_logfile (const char*) throw ();    

    /** Haeufigkeits-Statistik erstellen; 
      Arg1: Stream, in den das Ergebnis geschrieben werden soll */
    void write_intention_frequencies (std::ostream&) throw (std::bad_alloc);

    /** Haeufigkeits-Statistik erstellen; 
      Arg1: Stream, in den das Ergebnis geschrieben werden soll */
    void write_episode_frequencies (std::ostream&) throw (std::bad_alloc);

    /** Haeufigkeits-Statistik erstellen fuer Nachfolge fuer eine bestimmte Intention;
      Arg1: Stream, in den das Ergebnis geschrieben werden soll,
      Arg2: Intention, fuer das die Nachfolgeintention bestimmt werden sollen */
    void write_successor_frequencies (std::ostream&, const char*) throw (std::bad_alloc);
   
    /** Haeufigkeits-Statistik erstellen fuer Vorgaenger fuer eine bestimmte Intention;
      Arg1: Stream, in den das Ergebnis geschrieben werden soll,
      Arg2: Intention, fuer das die Vorgaengerintention bestimmt werden sollen */
    void write_predecessor_frequencies (std::ostream&, const char*) throw (std::bad_alloc);
   
    /** Die Zyklusnummern rausschreiben, in denen ein Behavior an war
      Arg2: Behavior, das untersucht werden soll, falls NULL, werden alle Behavior analysiert
      Arg3: Anfangszyklusnummer
      Arg4: Intervall-Laenge, die untersucht werden soll (0=alles) */
    void write_cycles (std::ostream&, const char*, unsigned int =0, unsigned int =0) throw ();

    /** Alle Behavior der zeitlichen Reichenfolge nach auflisten */
    void write_listall (std::ostream&) throw ();

    /** Behavior-Hierarchie weglassen (false) oder hinzufuegen (true) */
    void strip_hierarchy (bool) throw (std::bad_alloc);
  
    /** Uebergangsgraph zeichnen, Arg1: Dateiname (Praefix) fuer Zieldateien, Arg2: Liste der zu beruecksichtigenden Behaviors (leer=alle) */
    void transition_graph (const char*, const char*) throw (std::bad_alloc);

    /** naechste Standardsituation zum Behavior Arg2 nach maximal Arg3 Iterationen */
    void next_standard_frequencies (std::ostream&, const char*, const unsigned int) throw (std::bad_alloc);
  };

}
    
#endif
