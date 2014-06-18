
#ifndef Journal_h
#define Journal_h

#include <queue>
#include <string>
#include <fstream>
#include <sstream>
#include "../Fundamental/ConfigReader.h"
#include "TribotsException.h"

namespace Tribots {

  /** Klasse, um Fehler- und Warn- und sonstige Meldungen des Programms zu verwalten 
      Es gibt zwei Modi: 1. Speichermodus --> alle Meldungen werden im Hauptspeicher aufgesammelt 
                            und erst auf explizite Anweisung in einen Ausgabestream geschrieben
                         2. Streammodus --> alle Meldungen werden direkt in den Ausgabestream geschrieben
      Es gibt verschiedene Filtermoeglichkeiten: 1. nur Fehlermeldungen bearbeiten, alle anderen Meldungen ignorieren
                                                 2. Fehler- und Warnmeldungen bearbeiten, sonstige Meldungen ignorieren
                                                 3. alle Meldungen bearbeiten
      Typische Kombinationen: Speichermodus/nur Fehlermeldungen bei zeitkritischer Ausfuehrung
                              Streammodus/Fehler- und Warnmeldungen zum Debuggen
                              Streammodus/alle Meldungen um detailliert den Programmverlauf nachzuvollziehen
  */ 
  class Journal {
  private:
    int verbosity;                    ///< 1, 2, 3 siehe Filtermodi in Klassenbeschreibung
    std::queue<std::string> message_buffer;     ///< Schlange, um Meldungen zu speichern
    std::ostream* output_stream_pointer;   ///< Ausgabestream fuer Streammodus, bei Speichermodus NULL
    std::ofstream* output_file_pointer;    ///< Ausgabestream, falls Ausgabe direkt in eine Datei erfolgt
    std::string flush_output;              ///< Name der Ausgabedatei, in die zum Schluss im Speicher verbliebene Nachrichten geschrieben werden sollen (leer=stderr)
  public:
    /** Konstruktor, setze Speichermodus, setze Filtermodus 1 */
    Journal () throw ();
    /** Destruktor */
    ~Journal () throw ();

    /** Ausgabepuffer vollstaendig ausgeben */
    void flush () throw ();
        
    /** setze den Modus gemaess Parameterliste; 
        werte dazu die Zeilen aus:
          journal_verbosity= n
          journal_output= f
        wobei n=1,2,3 den Filter festlegt 
        und f den Modus: "memory" fuer Speichermodus, "stdout", "stderr" fuer Ausgabe in
          stdout bzw. stderr. Sonstige Angaben werden als Dateinamen interpretiert, in
          denen gespeichert werden soll */
    void set_mode (const ConfigReader&) throw (InvalidConfigurationException);
    /** setze den Filtermodus */
    void set_verbosity (int) throw ();
    /** setze den Speichermodus */
    void set_memory_mode () throw (); 
    /** setze den Streammodus, uebergebe Ausgabestream */
    void set_stream_mode (std::ostream&) throw ();
    /** den Puffer loeschen */
    void clear_buffer () throw ();
    /** den Pufferinhalt in einen Stream schreiben und dabei den Puffer loeschen */
    void write_buffer (std::ostream&) throw ();
    
    /** schreibe Fehlermeldung, Argumente: Quelldatei (.cc), Zeilennummer, beschreibender Text */
    void error (const char*, unsigned int, const char*) throw ();
    /** schreibe Warnung, Argumente wie bei 'error' */
    void warning (const char*, unsigned int, const char*) throw ();
    /** schreibe sonstige Meldung, Argument: beschreibender Text, Arg2: wenn true, wird ein Zeitstempel geschrieben */
    void message (const char*, bool =false) throw ();
    /** schreibe text in eine Datei zur SoundAusgabe*/
    void sound_message (const char*) throw ();    

    static Journal the_journal;
  };

}


// Makros, um auf einfache Weise Fehler, Warnungen oder Nachrichten zu erzeugen:
#define JERROR(t) Tribots::Journal::the_journal.error(__FILE__, __LINE__,t)
#define JWARNING(t) Tribots::Journal::the_journal.warning(__FILE__, __LINE__,t)
#define JMESSAGE(t) Tribots::Journal::the_journal.message(t, false)
#define JMESSAGETS(t) Tribots::Journal::the_journal.message(t, true)
#define JSPEECHOUT(t) Tribots::Journal::the_journal.sound_message(t)

#endif

