
#ifndef tribots_visible_object_h
#define tribots_visible_object_h

#include "../Fundamental/Time.h"
#include "../Fundamental/Vec.h"
#include <vector>

namespace Tribots {

  /** Struktur, um ein einzelnes erkanntes Object zu beschreiben 
      alle Angaben in ROBOTERKOORDINATEN */
  struct VisibleObject {
    enum ObjectType {
      unknown,      ///< object_type = unbekannt
      ball,         ///< object_type = Ball
      blue_goal,    ///< object_type = blaues Tor
      yellow_goal,  ///< object_type = gelbes Tor
      blue_pole,    ///< object_type = blau-gelb-blauer Eckpfosten
      yellow_pole,  ///< object_type = gelb-blau-gelber Eckpfosten
      teammate,     ///< object_type = Roboter der eigenen Mannschaft
      opponent,     ///< object_type = Roboter der gegnerischen Mannschaft
      obstacle,     ///< object_type = Hindernis (allgemein)
      white_line,   ///< object_type = weisse Linie
      blue_goal_post_left,     ///< object_type = blaues Tor, linker Pfosten
      yellow_goal_post_left,   ///< object_type = gelbes Tor, linker Pfosten
      blue_goal_post_right,    ///< object_type = blaues Tor, rechter Pfosten
      yellow_goal_post_right,  ///< object_type = gelbes Tor, rechter Pfosten
      ball3d                   ///< object_type = 3d-Ballposition
    };
    
    /** Default-Konstruktor */
    VisibleObject () throw ();
    /** Konstruktor
        Arg1: pos
        Arg2: object_type 
        Arg3: Breite des Objektes (nur fuer Hindernisse)
        Arg4: z-Koordinate des Objektes (nur ball3d) */
    VisibleObject (Vec, ObjectType =unknown, double =0, double =0) throw ();
    /** Copy-Konstruktor */
    VisibleObject (const VisibleObject&) throw ();
    /** Zuweisungs-Operator */
    const VisibleObject& operator= (const VisibleObject&) throw ();
    
    Vec pos;            ///< Position des Referenzpunktes des Objektes im Roboter-Koordinatensystem
    ObjectType object_type;    ///< Objektart, Typ (z.B. Ball, Hindernis, blaues Tor, gelbes Tor, usw.)
    double width;       ///< Breite des Objektes quer zur Blickrichtung in mm; ACHTUNG: wird nur fuer Hindernisse verwendet!
    double z;           ///< z-Position, wird nur fuer ball3d verwendet
    
    // Erweiterungen moeglich, z.B.:
    //  - Sicherheitswert, mit dem Objekt erkannt wurde
    //  - genauere geometrische Objektbeschreibung des Objektumfangs
  };


  /** Listenstruktur fuer erkannte Objekte mit Zeitstempel */
  struct VisibleObjectList {
    Time timestamp;                         ///< Zeitstempel: angenommener Zeitpunkt, zu der Bild aufgenommen wurde
    std::vector<VisibleObject> objectlist;  ///< Objektliste

    VisibleObjectList (unsigned int len =0) throw () : objectlist(len) {;}
    ~VisibleObjectList () throw () {;}
    VisibleObjectList (const VisibleObjectList&) throw (std::bad_alloc);
    const VisibleObjectList& operator= (const VisibleObjectList&) throw (std::bad_alloc);

    /** Writes a ascii serialization of the object to a stream. 
       Arg1: stream to write on. 
    **/
    void writeAt(std::ostream &stream) const;

    /** reads a ascii serialization of the object from a stream.
     Arg1: stream to read from.
     Returns number of correct read obstacles.
    **/
    int  readFrom(std::istream &stream);
  };
  
}

#endif

