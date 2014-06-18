
#ifndef _Tribots_CondensationFilter_h_
#define _Tribots_CondensationFilter_h_

#include "SelfLocalisation.h"
#include "../../Fundamental/Vec.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Structures/VisibleObject.h"
#include "../../Structures/RobotLocation.h"
#include "../Orga/OdometryContainer.h"
#include "FieldLUT.h"
#include <vector>
#include <fstream>       // fuer debug-Ausgabe

namespace Tribots {

  // ACHTUNG: Filter verwendet ein eigenes Koordinatensystem unabhaengig von der Spielrichtung.
  //          Ursprung ist der Spielfeldmittelpunkt
  //          die positive y-Achse weist in Richtung des blauen Tores

  /** struct Particle modelliert eine moegliche Roboterposition */
  struct Particle {
    Particle () throw ();
    Particle (const Particle&) throw ();
    const Particle& operator= (const Particle&) throw ();
    Vec pos;                 ///< Position in Spielfeldkoordinaten
    Angle heading;           ///< Ausrichtung in Spielfeldkoordinaten
    double probability;      ///< Wahrscheinlichkeit
  };


  /** Klasse CondensationFilter realisiert einen ParticleFilter fuer die Selbstlokalisierung,
      spezialisiert auf die Robocup-Domaene */
  class CondensationFilter : public SelfLocalisation {
  private:
    const OdometryContainer& odobox;        ///< Referenz auf Odometriewerte
    const FieldGeometry& field_geometry;      ///< Referenz auf Feldgeometrie
    std::vector<Particle>* particle_set;      ///< der aktuelle Satz an Partikeln
    FieldLUT fieldlut;                        ///< Tabelle fuer Abstaenden zu Linien
    double probability_line_sdev;             ///< Breite der Wahrscheinlichkeitsverteilung fuer die Abstaende zu Linien
    double probability_line_min;              ///< minimale "Wahrscheinlichkeit" einer Linien-Sensorinformation
    unsigned int min_set_size;                ///< minimale Anzahl Partikel
    unsigned int max_set_size;                ///< maximale Anzahl Partikel
    unsigned int num_new_particles;           ///< Anzahl in jeder Iteration zufaellig neu eingestreuter Partikel
    double spread_dev;                        ///< Standardabweichung fuer Partikelstreuen
    double spread_rot_dev;                    ///< Standardabweichung fuer Winkelstreuen
    unsigned int max_cycle_max_particle;      ///< Anzahl Zyklen mit maximaler Partikelzahl, zu der ein Neustreuen eingeleitet wird
    unsigned int max_cycle_counter;           ///< Zaehler fuer  max_cycle_max_particle
    Particle latest_average_pos;              ///< die letzte Durchschnittsposition
    double trimrate;                          ///< Anteil der Beobachtungen, die ignoriert werden (trimmed estimator)
    bool ignore_goals;                        ///< ignoriere Tor-Information?
    Time latest_update;                ///< Zeitstempel fuer letzten Aktualisierungsschritt
    std::ofstream* plog;                      ///< fuer debug-Ausgabe, erzeugt .cfpos-Datei mit Partikelpositionen

    unsigned int max_lines;                    ///< maximale Anzahl zu beruecksichtigender Liniensegmente
  public:
    /** Konstruktor */
    CondensationFilter (const ConfigReader&, const OdometryContainer&, const FieldGeometry&) throw (std::bad_alloc, InvalidConfigurationException);
    /** Destruktor */
    ~CondensationFilter () throw ();

    /** reinitialisieren mit gleichverteilten Partikeln */
    void reset () throw ();
    /** reinitialisieren mit Partikeln um einen Punkt arg1 herum */
    void reset (Vec) throw ();
    /** reinitialisieren mit Partikeln um einen Punkt arg1 mit Ausrichtung arg2 herum */
    void reset (Vec, Angle) throw ();

    /** Hinweis geben fuer SL auf Spielfeldsymmetrie, Arg1: ungefaehre Spielerposition; nicht implementiert! */
    void slMirrorHint (Vec) throw () {;}

    /** einen Aktualisierungsschritt durchfuehren
      arg1: erkannte weise Linien
      arg2: erkannte Hindernisse
      arg3: erkannte Tore
      ret: wurde die interne Repraesentation tatsaechlich veraendert, d.h. wurde Bildinformation eingearbeitet? */
    bool update (const VisibleObjectList&, const VisibleObjectList&, const VisibleObjectList&) throw ();
    /** Roboterposition und -geschwindigkeit fuer Zeitpunkt arg1 prognostizieren;
        Qualitaetswert bezieht sich nur auf die Qualitaet des letzten Sensorintegrationsschritts
        Angaben im SL-KOORDINATENSYSTEM */
    RobotLocation get (Time) const throw();

  protected:
    // einige Hilfsfunktionen:
    /** berechne die Summe der Wahrscheinlichkeiten */
    double sum_particle_probabilities () const;
    /** gleichverteilte Partikelmenge der Groesse arg1 erzeugen */
    void create_random_particleset (unsigned int);
    /** lokal um arg1 gestreute Partikelmenge der Groesse arg2 erzeugen */
    void create_local_particleset (const Vec, unsigned int);
    /** lokal um arg1 mit Ausrichtung arg2 gestreute Partikelmenge der Groesse arg3 erzeugen */
    void create_local_particleset (const Vec, const Angle, unsigned int);
    /** Partikelset resamplen, arg1=Groesse des Sets */
    void resample_particleset (unsigned int);
    /** bewege und streue Partikelmenge; 
        arg1=Translation in ROBOTERKOORDINATEN,
        arg2=Rotation,
        arg3=Streuung (Standardabweichung) fuer Position,
        arg4=Streuung fuer Rotation
        arg5=Anzahl Partikel, die rein zufaellig gleichverteilt erzeugt werden sollen
        arg6=Vorzugsrichtung der rein zufaellig gesampleten Partikel */
    void move_and_spread_particleset (const Vec, const Angle, double, double, unsigned int, const Angle&);
    /** die visuelle Information einbeziehen: weisse Linien */
    void evaluate_lines (const VisibleObjectList&);
    /** die visuelle Information einbeziehen: Tore */
    void evaluate_goals (const VisibleObjectList&);
    /** gewichteten Mittelwert der Partikelmenge berechnen;
        arg1: liefert die Determinante der Kovarianzmatrix der Positionen zurueck
        arg2: liefert die normierte Laenge des Richtungsvektors der mittleren Richtung zurueck
        arg3: ignoriere die letzten arg3 Partikel */
    Particle average_particle (double&, double&, unsigned int =0);
    /** Anpassung der Anzahl Partikel in Abhaengigkeit von der Streuung 
        arg1, arg2 sind die Werte, die von average_particle geliefert werden */
    unsigned int adapt_particle_number (double, double);

    std::vector<Particle>* reserve_set;       ///< interner Partikelsatz zum Resamplen
    std::vector<double>* select_numbers;      ///< internes Array zum Resamplen

  };

}

#endif
