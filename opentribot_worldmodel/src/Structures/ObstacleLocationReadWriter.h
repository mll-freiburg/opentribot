
#ifndef _Tribots_ObstacleLocationReadWriter_h_
#define _Tribots_ObstacleLocationReadWriter_h_

#include <iostream>
#include "ObstacleLocation.h"

namespace Tribots {

  /** Klasse, um ObstacleLocation zu schreiben */
  class ObstacleLocationWriter {
  public:
    /** Konstruktor; Arg1=Zielstream, Arg2=binaeres Format? */
    ObstacleLocationWriter (std::ostream&, bool = true) throw ();
    /** Destruktor, fuehrt flush aus */
    ~ObstacleLocationWriter () throw ();
    /** schreibe ein ObstacleLocation;
        Arg1: Zeitstempel in ms
        Arg2: eigentlicher ObstacleLocation */
    void write (unsigned long int, const ObstacleLocation&) throw ();
  private:
    std::ostream& dest;
    bool binary_mode;
  };


  /** inverse Klasse zu ObstacleLocationWriter */
  class ObstacleLocationReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    ObstacleLocationReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem der naechste ObstacleLocation vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel (Rueckgabe)
        Arg2: letzter gelesener ObstacleLocation (Rueckgabe)
        Arg3: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, ObstacleLocation&, unsigned long int) throw ();
  private:
    std::istream& src;
    unsigned long int next;
    unsigned int encoding_type;  // 0=ascii, 1=binaer

    // fuer binaeres Lesen:
    unsigned long int old_tav;
    int old_player;
    unsigned long int binary_read_old ();
  };

}


#endif
