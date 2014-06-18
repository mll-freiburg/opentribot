
#ifndef tribots_obstacle_location_h
#define tribots_obstacle_location_h

#include "../Fundamental/Vec.h"
#include <vector>
#include <iostream>

namespace Tribots {

  /** Struktur, um ein hindernis zu modellieren */
  struct ObstacleDescriptor {
    Vec pos;  ///< Position in Weltkoordinaten
    double width;  ///< Breite des Hindernisses
    int player;  ///< Spielernummer; negativ, wenn nicht bekannt
    Vec velocity;  ///< Hindernisbewegungsgeschwindigkeit
    ObstacleDescriptor () throw ();
    ObstacleDescriptor (const ObstacleDescriptor&) throw ();
    const ObstacleDescriptor& operator= (const ObstacleDescriptor&) throw ();
  };
  /** Struktur, um die Hindernisse auf dem Spielfeld zu repraesentieren 
      alle Koordinaten (in WELTKOORDINATEN) sind relativ zur aktuellen Spielrichtung:
      (0,0) ist der Mittelpunkt des Feldes
      die y-Achse zeigt in Richtung des gegnerischen Tores
      die x-Achse zeigt rechtwinklig dazu (rechtshaendisches Koordinatensystem)
      ein Winkel von 0 bezeichnet die Parallele zur x-Achse
      alle Laengenangaben in mm, alle Winkelangaben in rad 
      alle Geschwindigkeiten in m/s bzw. rad/s */
  struct ObstacleLocation : public std::vector<ObstacleDescriptor> {
    /** Standardkonstruktor, uebergeben wird die Anzahl Hindernisse */
    ObstacleLocation (unsigned int =0) throw (std::bad_alloc);
    /** Copy-Konstruktor */
    ObstacleLocation (const ObstacleLocation&) throw (std::bad_alloc);

    /** Writes a ascii serialization of the object to a stream. 
        Arg1: stream to write on. **/
    void writeAt(std::ostream &stream) const;

    /** reads a ascii serialization of the object from a stream.
     Arg1: stream to read from.
     Returns number of correct read obstacles. **/
    int  readFrom(std::istream &stream);
  };

}

#endif

