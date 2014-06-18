
#ifndef Tribots_OdometrySlidingWindowBallFilter_h
#define Tribots_OdometrySlidingWindowBallFilter_h

#include "BallFilter.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/DynamicRingBuffer.h"
#include "../Orga/OdometryContainer.h"
#include <fstream>

namespace Tribots {

  /** Klasse OdometrySlidingWindowBallFilter modelliert eine einfache Ballverfolgung und 
      Geschwindigkeitsschaetzung; Das Verfahren basiert auf der Bildung eines
      kleinste-Quadrate-Schaetzers ueber die letzten n gesehenen Ballpositionen
      bei linearem Bewegungsmodell (gleichfoermige Bewegung); n wird automatisch angepasst 
      beruecksichtigt die Verschiebung des Roboters gemaes Odometrie, nicht gemaes Selbstlokalisierung */
  class OdometrySlidingWindowBallFilter : public BallFilter {
  public:
    /** erzeuge BallFilter; lese Parameter aus arg1 */
    OdometrySlidingWindowBallFilter (const ConfigReader&, const OdometryContainer&) throw (std::bad_alloc);
    /** Destruktor */
    ~OdometrySlidingWindowBallFilter () throw ();
    /** uebergebe gemessene Position an Filter; 
        arg1: geshene Ballposition
        arg2: Roboterposition und -ausrichtung zum Zeitpunkt der Bildinformation
        ret: wurde wirklich eine Aktualisierung vorgenommen? */
    bool update (const VisibleObjectList&, const RobotLocation&) throw ();
    /** liefere mutmassliche Position und Geschwindigkeit zum Zeitpunkt arg1 */
    BallLocation get (const Time) const throw ();
  private:
    /** Paar aus Ballposition und Zeitstempel */
    struct PairTimePos {
      Vec pos;         ///< Position      
      Time timestamp;  ///< Zeitstempel
    };
    const OdometryContainer& odobox;           ///< Odometrieverwaltung
    DynamicRingBuffer<PairTimePos> pos_list;   ///< Liste alter Zeitstempel (Ringpuffer)

    bool first_call;                           ///< true vor dem ersten Aufruf von new_position
    unsigned int max_buffer_size;              ///< maximale Groesse des Ringpuffers
    unsigned int min_buffer_size;              ///< minimale Groesse des Ringpuffers

    Time ref_time;               ///< Referenz-Zeitpunkt = Zeitpunkt der letzten Bildinformation
    RobotLocation ref_rpos;      ///< Referenz-Position des Roboters
    Vec ref_bpos;                ///< Referenz-Position des Balls in Roboterkoordinaten
    Vec ref_bvel;                ///< Referenz-Geschwindigkeit des Balls in Roboterkoordinaten

    Time latest_motion_model;    ///< Zeitpunkt, zu dem letztmalig ein gueltiges Bewegungsmodell bestimmt wurde

    double latest_error;         ///< Abweichung prognostizierte/gemessene Ballposition bei der vorangegangenen Aktualisierung
    double max_error;            ///< maximal erlaubter Fehler
    bool ball_raised;            ///< ist true, falls zuvor erkannt wurde, dass der Ball angehoben wurde

    double raised_hysterese_lower2;     ///< oberer Schwellwert der Geschwindigkeit zum Quadrat fuer "raised-Analyse"
    double raised_hysterese_upper2;     ///< unterer Schwellwert der Geschwindigkeit zum Quadrat fuer "raised-Analyse"

    std::ofstream* plog;         ///< Ausgabestream fuer Debugen

    bool update_motion_model () throw ();                     ///< berechnete Refernz-Ballposition und -geschwindigkeit neu, liefert true falls Berechnung numerisch stabil
    Vec get_relative_ball_prediction (Time) const throw ();   ///< Position relativ zu Referenzposition berechnen
    void rescale_ring_buffer (unsigned int) throw ();   ///< Ringpuffer auf die Groesse arg1 verkleinern (arg1>=1)
    bool inside_field (Vec) const throw ();   ///< prueft, ob die Position innerhalb des Spielfeldes (einschlieslich Rand) liegt
  };

}

   
#endif
