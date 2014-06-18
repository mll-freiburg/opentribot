
#ifndef tribots_control_loop_decision_h
#define tribots_control_loop_decision_h

#include "../Robot/Robot.h"
#include "../Player/Player.h"
#include "../WorldModel/WorldModel.h"
#include "../ImageProcessing/Vision.h"
#include "../UserInterface/UserInterface.h"
#include <time.h>


/** Namespace Tribots enthaelt alle neuen Softwareteile fuer die Tribots */
namespace Tribots {

  /** Implementierung der zentralen Rgelschleife
      Aufgaben: Initialisierung der Komponenten, Kontrollfluss, Datenfluss, 
                Zeitkontrolle, Start-/Stopp-Abfrage von Tastatur */
  class ControlLoopVision {
  private:
    Robot* the_robot;
    Player* the_player;
    WorldModel* the_world_model;
    Vision* the_vision;
    UserInterface* the_user_interface;
    const ConfigReader& configuration_list;

    Time timestamp;           ///< Attribut, um die Schleifendurchlaeufe zu takten
    long int loop_time;       ///< Laenge eines Regelintervalls in ms
    Time first_cycle_time;    ///< Startzeitpunkt der Regelschleife
    Time stopping_time;       ///< Stoppzeit fuer Programm (nur fuer Testzwecke)
    bool stop_at_stopping_time;  ///< true, wenn zu Stoppzeitpunkt das Programm angehalten werden soll

    bool report_computation_time;              ///< Gesamt-Rechenzeiten erfassen?
    bool report_computation_time_detailed;     ///< Rechenzeit in jeder Iteration erfassen
    bool report_computation_time_gnuplot;      ///< Rechenzeit in jeder Iteration fuer gnuplot erfassen
    unsigned long int usec_vision;             ///< bisherige Rechenzeit fuer Bildverarbeitung
    unsigned long int usec_world_model;        ///< bisherige Rechenzeit fuer Welt-Modell (update-Funktion)
    unsigned long int usec_player;             ///< bisherige Rechenzeit fuer Entscheidungsfindung
    unsigned long int usec_robot;              ///< bisherige Rechenzeit fuer Roboteransteuerung
    unsigned long int usec_user_interface;     ///< bisherige Rechenzeit fuer User Interface und Kommunikation
    unsigned long int usec_idle;               ///< bisherige Wartezeit
    unsigned long int num_cycles;              ///< Anzahl Iterationen

    clock_t lastUserTime;
    unsigned long int lastSystemTime;
    unsigned long int lastChildrenUserTime;
    unsigned long int lastChildrenSystemTime;

    unsigned int cpuMhz ();   ///< aus /proc/cpuinfo die Prozessor-Geschwindigkeit lesen, liefert 0, wenn Fehler auftritt
  
    bool restart_log;
  public:
    /** Konstruktor */
    ControlLoopVision (const ConfigReader&, bool = false) throw (TribotsException, std::bad_alloc);
    /** Destruktor */
    ~ControlLoopVision () throw ();
    /** starte Hauptschleife */
    void loop () throw ();
  };

}

#endif

