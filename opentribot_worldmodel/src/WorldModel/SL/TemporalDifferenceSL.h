
#ifndef _Tribots_TemporalDifferenceSL_h_
#define _Tribots_TemporalDifferenceSL_h_

#include "SelfLocalisation.h"
#include "FieldLUT.h"
#include "../Orga/OdometryContainer.h"
#include "VisualPositionOptimiser.h"
#include "RobotPositionKalmanFilter.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/RingBuffer.h"


namespace Tribots {

  /** Selbstlokalisierung aehnlich ErrorMinimiserSL, aber staerkere Betonung der 
       zeitlichen Konsistenz der Positionshypothesen; benoetigt zuverlaessigere Odometrie */
  class TemporalDifferenceSL : public SelfLocalisation {
  protected:
    /** Repraesentation einer Positions-Hypothese */
    struct AlternativePose {
      AlternativePose ();
      void init_randomly ();
      void init_position (Vec);
      void init_pose (Vec, Angle);

      unsigned long int num_iterations_alive;  ///< Anzahl Iterationen, die diese Hypothesebereits geprueft wird
      unsigned long int num_iterations_visual_input;   ///< Anzahl Iterationen, zu denen Linien vorlagen
      double way_alive;  ///< Laenge des zurueckgelegten Wegs in mm, seit dem diese Hypothese geprueft wird
      bool reset_hypothesis;  ///< true, wenn diese Hypothese gleichrangig mit Haupthypothese behandelt werden soll

      RobotPositionKalmanFilter kfilter;  ///< Kalman-Filter beinhaltet Position, Ausrichtung
      double visual_error;  ///< Mass dafuer, in wie weit die Bildinformation zur Hypothese passen, geglaettet ueber Zeit
      double odo_vis_mismatch_linear;  ///< Diskrepanz von Bildposition und Odometrieposition in mm, geglaettet ueber Zeit
      double odo_vis_mismatch_angular;  ///< Diskrepanz von Bildausrichtung und Odometrieausrichtung in rad, geglaettet ueber Zeit

      bool heading_checked;  ///< true, wenn die Tor-Orientierung bereits ueberprueft wurde
      double rotation_since_heading_check;  ///< rotierter Winkel (in rad) seit dem letzten Headingcheck
      double ema_right_goal;  ///< EMA-Kriterium, das die Tor-Ausrichtung bestaetigt
      double ema_wrong_goal;  ///< EMA-Kriterium, das die Tor-Ausrichtung widerlegt
      RingBuffer<Time> latestSLMirrorHintTimes;   ///< eine Liste mit den letzten Zeitpunkten, zu denen SLMirrorHint empfangen wurde

      double magic_number;  ///< die Gesamtevaluierungsguete gemaess einer "magischen Formel". Je kleiner, desto besser
    };

    const FieldGeometry& fg;
    FieldLUT* field_lut;  ///< Die Abstandstabelle
    VisualPositionOptimiser* vis_optimiser;  ///< Optimierungsroutine fuer Sensorinformation
    const OdometryContainer& odobox;  ///< Odometrieinformation

    unsigned int max_lines;  ///< maximale Anzahl zu beruecksichtigender Liniensegmente
    double max_line_distance;  ///< maximaler Abstand, bis zu dem Linien beruecksichtigt werden

    bool consider_yellow_goal;  ///< gelbes Tor beruecksichtigen?
    bool consider_blue_goal;  ///< blaues Tor beruecksichtigen?
    unsigned int num_internal_alternatives;  ///< fuer den Hauptfilter intern n Alternativpositionen berechnen?
    bool internal_random_direction;  ///< Sollen die internen Alternativen mit zufaelligen Richtungen initialisiert werden?
    bool do_remove_occluded_lines;  ///< durch Hindernisse verdeckte Linien unberuecksichtigt lassen?
    bool do_remove_occluded_lines_far;  ///< durch weit entfernte Hindernisse (>0.5m) verdeckte Linien unberuecksichtigt lassen?
    bool do_remove_lines_outside_field;  ///< Liniensegmente ausserhalb des Feldes entfernen?
    bool has_gyro;     ///< Gyroskop vorhanden?
    double main_to_reset_hysteresis_lower;  ///< Schranke, unterhalb derer die Haupthypothese zur Resethypothese werden soll
    double main_to_reset_hysteresis_upper;   ///< Schranke, oberhalb derer die Haupthypothese zur Resethypothese werden soll
    int prefered_enter;   ///< Angabe, auf welcher Spielfeldseite bevorzugt Roboter eingestellt werden: +1 rechts, -1 links, 0 neutral (jeweils vom gelben zum blauen Tor gesehen)

    unsigned int counter_internal_alternatives;  ///< Ein Zaehler, um interne Alternativen zyklisch durchzugehen

    AlternativePose main_hypothesis;  ///< Positions-Hypothese
    std::vector<AlternativePose> alternative_hypothesis;  ///< Alternative-Positionen
    unsigned int num_alternatives;  ///< Anzahl gewuenschter Alternativpositionen
    unsigned int num_alternative_updates;  ///< Anzahl Alternativpositionen, die pro Zyklus aktualisiert werden sollen
    unsigned int next_alternative;  ///< die Alternative, die als naechstes aktualisiert werden soll

    Time latest_main_update;  ///< Zeitstempel der letzten Aktualisierung der Haupthypothese (Bildzeitpunkt)
    Time latest_all_update;  ///< Zeitstempel der letzten Aktualisierung aller Hypothesen (Bildzeitpunkt)
    VisibleObjectList lines_all_update;  ///< die Liniensegemente, die fuer den letzten vollen Update verwendet werden sollen
    Vec odo_linear_all_update;  ///< die lineare Bewegung (mm, in Roboterkoordinaten), die fuer den letzten vollen Update verwendet werden soll
    Angle odo_angular_all_update;  ///< die Rotation (rad), die fuer den letzten vollen Update verwendet werden soll
    unsigned int cycweight_all_update;  ///< Fehlergewichtung zwischen jedes mal aktualisierter Haupthypothese und nur gelegentlich aktualisierter Alternative
    unsigned int cycweight;  ///< dito, zum Hochzaehlen
    double average_vision_delay;  ///< ungefaehre Zeit in msec, die zwischen Bildaufnahme und Auswertung ensteht
    bool valid_hypothesis;   ///< valid-Attribut

    /** eine Hypothese fortschreiben:
        arg1: Hypothese (Argument und Rueckgabewert)
        arg2: Liniensegmente
        arg3: zurueckgelegter Weg seit letztem Update
        arg4: Rotation seit letztem Update
        arg5: sollen interne Alternativen verwendet werden?
        arg6: auf wie viele Zyklen soll der Fehler verteilt werden? */
    void update_alternative (AlternativePose&, const VisibleObjectList&, Vec, Angle, bool, unsigned int =1);

    /** die globale Orientierung einer Hypothese pruefen (nach Torfarbe)
        arg1: Hypothese (Argument und Rueckgabewert)
        arg2: Tore
        arg3: gelbes Tor beruecksichtigen?
        arg4: blaues Tor beruecksichtigen?
        arg5: seit dem letzten Zyklus zurueckgelegte Rotation (in rad)
        return: wurde die Orientierung geaendert? */
    bool check_goals (AlternativePose&, const VisibleObjectList&, bool, bool, double);

    /** durch Hindernisse verdeckte Linien entfernen:
        arg1: Rueckgabe Linien ohne Verdeckungen
        arg2: Liste mit gesehenen Linien
        arg3: Liste mit gesehenen Hindernissen
        arg4: Maximalabstand in mm, bis zu dem Linien beruecksichtigt werden sollen
        arg5: Minimalabstand in mm, ab dem verdeckende Hindernisse beruecksichtigt werden sollen */
    void remove_occluded_lines (VisibleObjectList&, const VisibleObjectList&, const VisibleObjectList&, double, double);

    /** Linien ausserhalb des Spielfeldes entfernen:
        arg1: Rueckgabe Linien innerhalb des Feldes
        arg2: Liste mit gesehenen Linien
        arg3: Roboterposition
        arg4: Roboterausrichtung */
    void remove_lines_outside_field (VisibleObjectList&, const VisibleObjectList&, Vec, Angle);

    /** Alternativen untereinander und mit Haupthypothese vergleichen,
        bei zu grosser Aehnlichkeit oder Ueberalterung Alternativen zufaellig reinitialisieren,
        bei ueberzeugender Alternative die Haupthypothese ersetzen */
    void alternative_competition ();

    /** bewertet eine Alternative mit einer "magischen Funktion": je kleiner das Ergebnis, desto besser ist 
        die Alternative, gibt Hypothesen mit nicht gesetztem reset_pos-Attribut einen Malus. Setzt das
        magic_number-Attrribut des Arguments */
    double magic_alternative_evaluation (AlternativePose&);

  public:
    /** Konstruktor */
    TemporalDifferenceSL (const ConfigReader&, const OdometryContainer&, const FieldGeometry&) throw (std::bad_alloc);
    /** Destruktor */
    ~TemporalDifferenceSL () throw ();

    /** Position fortschreiben und mit Bildinformation abgleichen;
        arg1: erkannte weise Linien
        arg2: erkannte Hindernisse
        arg3: erkannte Tore
        ret: wurde die interne Repraesentation tatsaechlich veraendert, d.h. wurde Bildinformation eingearbeitet? */
    bool update (const VisibleObjectList&, const VisibleObjectList& , const VisibleObjectList&) throw ();

    /** Roboterposition und -geschwindigkeit fuer den Zeitpunkt arg1 prognostizieren; 
        Qualitaetswert bezieht sich nur auf die Qualitaet des letzten Sensorintegrationsschritts
        Angaben im SL-Koordinatensystem */
    RobotLocation get (Time) const throw ();

    /** Roboterposition zufaellig reinitialisieren */
    void reset () throw ();
    /** Roboterposition an Position arg1 reinitialisieren */
    void reset (Vec) throw ();
    /** Roboterposition an Position arg1 mit Ausrichtung arg2 reinitialisieren; 
        arg2 misst die Verdrehung des Roboterkoordinatensystems vom Weltkoordinatensystem */
    void reset (Vec, Angle) throw ();

    /** Hinweis geben fuer SL auf Spielfeldsymmetrie, Arg1: ungefaehre Spielerposition */
    void slMirrorHint (Vec) throw ();
  };

}

#endif
