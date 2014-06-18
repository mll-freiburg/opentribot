
#ifndef Tribots_EMAObstacleFilter_h
#define Tribots_EMAObstacleFilter_h

#include <deque>
#include "../../Structures/VisibleObject.h"
#include "../../Structures/ObstacleLocation.h"
#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/DynamicRingBuffer.h"


namespace Tribots {

  /** Hindernis-Filter auf Grundlage einer exponentiellen Glaettung;
      filtert kurzfristige Stoerungen in der Hinderniserkennung;
      (bis maximal 3 Frames Stoerung) */
  class EMAObstacleFilter {
  public:
    /** Konstruktor */
    EMAObstacleFilter (const ConfigReader&, const FieldGeometry&) throw ();
    /** Destruktor */
    ~EMAObstacleFilter () throw ();

    /** liefere Position der Hindernisse*/
    ObstacleLocation get () const throw ();
    /** liefere Position der Hindernisse einschliesslich der Eckpfosten */
    ObstacleLocation get_with_poles () const throw ();
    /** liefere Position der Hindernisse einschliesslich der Eckpfosten 
        und der Positionen, an denen der Roboter blockiert war */
    ObstacleLocation get_with_poles_and_stuck () const throw ();
    /** analysiere die Hindernisse anhand der visuellen Information;
        Arg1: Liste gesehener Hindernisse in Roboterkoordinaten
        Arg2: Roboterposition zu diesem Zeitpunkt
        Arg3: Ballposition zu diesem Zeitpunkt */
    void update (const VisibleObjectList&, const RobotLocation&, const BallLocation&) throw ();

  protected:
    struct TPos {
      Time t;
      Vec pos;
    };
    /** Struktur, um Hindernisse intern zu verwalten */
    struct ObstacleProperties {
      Vec pos;  ///< Position in Weltkoordinaten
      double width;  ///< Breite in mm
      double probability;  ///< "Wahrscheinlichkeit": ein Plausibilitaetswert
      bool active;  ///< true, wenn Hindernis ausgegeben wird bei Anfrage nach Hindernissen
      DynamicRingBuffer<TPos> recentPositions;  ///< Anker zeigt stets auf aeltesten Eintrag
      Vec velocity;  ///< Geschwindigkeitsvektor des Hindernisses
    };
    struct AssignmentProperties {
      Vec pos;  ///< Position in Weltkoordinaten
      double width;  ///< Breite in mm
      double nearest_dist;  ///< Fuer Assignment: Abstand zum naechstliegenden Objekt
      int nearest_index;  ///< Fuer Assignment: Index des naechstliegenden Objekt
      bool near_robot;  ///< true, wenn Hindernis naeher als 1m vom Roboter entfernt
    };
    std::deque<ObstacleProperties> obstacles;       ///< Liste vorhandener Hindernisse
    std::vector<AssignmentProperties> assignments;  ///< Assignment-Container fuer interne Berechnungen
    double ema;                                     ///< EMA-Glaettungsparameter (je kleiner, desto kurzfristiger)
    double hysterese_lower;                         ///< unterer Schwellwert der Hysterese
    double hysterese_higher;                        ///< oberer Schwellwert der Hysterese
    ObstacleLocation poles;                         ///< Liste der Eckpfosten
    unsigned int stuck_obstacle_delay;              ///< Zeit in ms, die Stuck-Hindernisse vorhanden sein sollen
    bool remove_ball_obstacles;                     ///< (scheinbare) Hindernisse vor dem Ball entfernen
    double min_obstacle_width;                      ///< minimale Hindernisbreite in mm
    unsigned int max_num_recent_pos;                ///< maximale Anzahl zu speichernder aelterer Positionen
  };

}

#endif
