
#ifndef TribotsTools_CycleContainer_h
#define TribotsTools_CycleContainer_h

#include <fstream>
#include <stdexcept>
#include <deque>
#include <map>
#include <sys/time.h>
#include "../components/CycleInfo.h"
#include "../../Structures/RobotLocationReadWriter.h"
#include "../../Structures/BallLocationReadWriter.h"
#include "../../Structures/ObstacleLocationReadWriter.h"
#include "../../Structures/GameStateReadWriter.h"
#include "../../Structures/VisibleObjectReadWriter.h"
#include "../../Structures/DriveVectorReadWriter.h"
#include "../../Structures/MessageBoardReadWriter.h"

namespace TribotsTools {

  /** Klasse, um die CycleInfo aus Dateien zu lesen und zu verwalten */
  class CycleContainer {
  private:
    std::ifstream* rloc_stream;              // Eingabestream fuer Roboterposition
    std::ifstream* bloc_stream;              // Eingabestream fuer Ballposition
    std::ifstream* oloc_stream;              // Eingabestream fuer Hindernisse
    std::ifstream* vloc_stream;              // Eingabestream fuer gesehene Objekte
    std::ifstream* log_stream;               // Eingabestream fuer Loginfo
    std::ifstream* ppos_stream;              // Eingabestream fuer Suchpositionen der Selbstlokalisierung
    std::ifstream* gs_stream;                // Eingabestream fuer Gamestates
    std::ifstream* dv_stream;                // Eingabestream fuer Fahrtvektoren
    std::ifstream* mbd_stream;               // Eingabestream fuer Messageboard

    unsigned int next_log_timestamp;         // naechster gelesener Timestamp fuer .log-Datei
    unsigned int next_ppos_timestamp;        // naechster gelesener Timestamp fuer Selbstlokalisierungs-Suchpositionen

    Tribots::RobotLocationReader* rloc_reader;
    Tribots::BallLocationReader* bloc_reader;
    Tribots::ObstacleLocationReader* oloc_reader;
    Tribots::VisibleObjectReader* vloc_reader;
    Tribots::GameStateReader* gs_reader;
    Tribots::DriveVectorReader* dv_reader;
    Tribots::MessageBoardReader* mbd_reader;

    std::deque<CycleInfo> cycle_queue;       // bisher gelesene Infos
    unsigned long int current_cycle;         // Nummer des aktuellen Zyklus
    std::string fname_praefix;               // Dateinamen-Praefix

    timeval reftime;             // Referenzzeitpunkt (Systemzeit)
    std::deque<std::pair<unsigned long int, unsigned int> > synch_signals;   // Synchronisationssignale

    bool read_next (CycleInfo&, unsigned long int) throw ();    // liest naechste Zyklen bis Nr arg2 von Datei; liefert bei Erfolg true

  public:
    /** Konstruktor, uebergeben wird der Basisname der Logdateien */
    CycleContainer (const char*) throw (std::invalid_argument);
    /** liefere die aktuelle Zyklusnummer */
    long int cycle_num () const throw ();
    /** weiterschalten um arg1 Iteration, falls moeglich 
        aktuelle Iterationsnummer wird zurueckgegeben */
    long int step (long int =1) throw ();
    /** einen bestimmten Zyklus aufrufen, falls moeglich 
        aktuelle Iterationsnummer wird zurueckgegeben */
    long int set (unsigned long int) throw ();
    /** eine bestimmte Programmzeit aufrufen, falls moeglich 
        aktuelle Iterationsnummer wird zurueckgegeben */
    long int set_time (unsigned long int) throw ();
    /** Infos der aktuellen Iteration holen, voausgesetzt es sind Infos da */
    const CycleInfo& get () const throw ();
    /** liefere Anzahl gespeicherter Zyklen */
    unsigned int size() const throw ();
    /** Datei neu laden */
    bool revert ();
    /** Datei neu laden */
    bool revert (const char*);
    /** aktuelle Zyklusinfo ersetzen */
    void replace (const CycleInfo&);
    /** Referenzzeitpunkt abfragen */
    timeval get_reference_time () const throw ();
    /** Referenzzeitpunkt setzen */
    void set_reference_time (const timeval&) throw ();

    /** Hilfsfunktion: Zeit umrechnen:
      Arg1: Zeit in Millisekunden bezueglich 
      Arg2: Referenzzeitpunkt (Nullpunkt), auf den sich Arg1 bezieht,
      Return: Zeit in Millisekunden bezueglich des internen Referenzzeitpunktes (reftime) */
    long int shift_time (long int, timeval) const throw ();

    /** Liste der Synchronisationssignale liefern */
    const std::deque<std::pair<unsigned long int, unsigned int> >& get_synchronisation_signals () throw ();
  protected:
    /** Logdateien oeffnen, liefert true bei Erfolg */
    bool open_files (const char* praefix);
    /** Logdateien schliessen */
    void close_files ();
  };

}

#endif

