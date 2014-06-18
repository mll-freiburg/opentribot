
#ifndef Tribots_ErrorMinimiserSL_h
#define Tribots_ErrorMinimiserSL_h

#include "FieldLUT.h"
#include "../Orga/OdometryContainer.h"
#include "VisualPositionOptimiser.h"
#include "RobotPositionKalmanFilter.h"
#include "../../Fundamental/ConfigReader.h"
#include "SelfLocalisation.h"


namespace Tribots {

  /** Selbstlokalisierung mit Hilfe von (a) einem Fehlerminimierungsansatz auf der Bildinformation,
      (b) einem Kalman-Filter zum Verfolgen der Position sowie (c) expliziten Alternativpositionen */
  class ErrorMinimiserSL : public SelfLocalisation {
  protected:
    /** Struktur, um alternative Positionen zu verwalten */
    struct AltPos {
      AltPos ();                               ///< Zufallsinitialisierung
      AltPos (Vec);                            ///< Position gegeben, usrichtung zufaellig
      AltPos (Vec, Angle);                     ///< Position und Orientierung gegeben
      RobotPositionKalmanFilter rpos_filter;   ///< Kalman-Filter fuer Roboterposition und Geschwindigkeit
      Time init_time;                          ///< Zeitpunkt der Erzeugung
      double winning_coefficient;              ///< Zahl, die ein Mass fuer die Qualitaet der Alternative, indem verglichen wird, ob die Alternative besser als andere ist
      double latest_error;                     ///< visueller Fehler bei der letzten Iteration
      bool heading_checked;                    ///< true, wenn die Tor-Orientierung bereits ueberprueft wurde
      double ema_right_goal;                   ///< EMA-Kriterium, das die Tor-Ausrichtung bestaetigt
      double ema_wrong_goal;                   ///< EMA-Kriterium, das die Tor-Ausrichtung widerlegt
    };

    FieldLUT* field_lut;                       ///< Die Abstandstabelle
    VisualPositionOptimiser* vis_optimiser;    ///< Optimierungsroutine fuer Sensorinformation
    const OdometryContainer& odobox;           ///< Odometrieinformation
    Time latest_update;                        ///< Zeitstempel der letzten Aktualisierung
    unsigned int cycles_since_reset;           ///< Anzahl Zyklen seit dem letzten Reset

    unsigned int max_lines;                    ///< maximale Anzahl zu beruecksichtigender Liniensegmente
    double max_line_distance;                  ///< maximaler Abstand, bis zu dem Linien beruecksichtigt werden

    bool consider_yellow_goal;                 ///< gelbes Tor beruecksichtigen?
    bool consider_blue_goal;                   ///< blaues Tor beruecksichtigen?
    bool use_internal_alternatives;            ///< fuer den Hauptfilter intern vier Alternativpositionen berechnen?
    bool do_remove_occluded_lines;             ///< durch Hindernisse verdeckte Linien unberuecksichtigt lassen?
    bool do_remove_lines_outside_field;        ///< Liniensegmente ausserhalb des Feldes entfernen?
    bool has_gyro;     ///< Gyroskop vorhanden?

    AltPos main_position;                      ///< derzeitige Positions-Hypothese
    unsigned int num_external_alternatives;    ///< Anzahl Alternativhypothesen
    std::vector<AltPos> external_alternatives; ///< Array mit Alternativpositionen

    void update_alternative (AltPos&, const VisibleObjectList&, const VisibleObjectList&, bool, bool);   ///< Fortschreiben einer Position (arg1) mit Liniensegmenten (arg2) und Toren (arg3). arg4 gibt an, ob interne Alternativen verwendet werden sollen, arg5 ist true, wenn es sich um die Hauptalternative handelt, sonst false
    void remove_occluded_lines (VisibleObjectList&, const VisibleObjectList&, const VisibleObjectList&, double =6000);  ///< durch Herausloeschen verdeckter Linien aus (arg2) die Liste (arg1) generieren. (arg3)=Hindernisse. (arg4)=Maximaldistanz, bis zu der Linien ueberhaupt beruecksichtigt werden sollen
    void remove_lines_outside_field (VisibleObjectList&, const VisibleObjectList&, Vec, Angle);  ///< Linien ausserhalb des Feldes entfernen; arg1=(return) Liste mit Linien innerhalb des Feldes, arg2=Originalliste mit allen Linien, arg3=Roboterposition, arg4=Roboterausrichtung
  public:
    /** Konstruktor */
    ErrorMinimiserSL (const ConfigReader&, const OdometryContainer&, const FieldGeometry&) throw (std::bad_alloc);
    /** Destruktor */
    ~ErrorMinimiserSL () throw ();

    /** Position fortschreiben und mit Bildinformation abgleichen;
        arg1: erkannte weise Linien
        arg2: erkannte Hindernisse
        arg3: erkannte Tore
        ret: wurde die interne Repraesentation tatsaechlich veraendert, d.h. wurde Bildinformation eingearbeitet? */
    bool update (const VisibleObjectList&, const VisibleObjectList& , const VisibleObjectList&) throw ();

    /** Roboterposition und -geschwindigkeit fuer den Zeitpunkt arg1 prognostizieren; 
        Qualitaetswert bezieht sich nur auf die Qualitaet des letzten Sensorintegrationsschritts
        Angaben im SL-KOORDINATENSYSTEM */
    RobotLocation get (Time) const throw ();

    /** Roboterposition zufaellig reinitialisieren */
    void reset () throw ();
    /** Roboterposition an Position arg1 reinitialisieren */
    void reset (Vec) throw ();
    /** Roboterposition an Position arg1 mit Ausrichtung arg2 reinitialisieren; 
        arg2 misst die Verdrehung des Roboterkoordinatensystems vom Weltkoordinatensystem */
    void reset (Vec, Angle) throw ();

    /** Hinweis geben fuer SL auf Spielfeldsymmetrie, Arg1: ungefaehre Spielerposition; nicht implementiert! */
    void slMirrorHint (Vec) throw () {;}
  };

}

#endif
