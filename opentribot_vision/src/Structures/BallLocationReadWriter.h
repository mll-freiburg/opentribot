
#ifndef Tribots_BallLocationReadWriter_h
#define Tribots_BallLocationReadWriter_h

#include <iostream>
#include "BallLocation.h"

namespace Tribots {

  /** Klasse, um BallLocation zu schreiben */
  class BallLocationWriter {
  public:
    /** Konstruktor; Arg1=Zielstream, Arg2=binaeres Format? */
    BallLocationWriter (std::ostream&, bool = true) throw ();
    /** Destruktor, fuehrt flush aus */
    ~BallLocationWriter () throw ();
    /** schreibe ein BallLocation;
        Arg1: Zeitstempel in ms des schreibens
        Arg2: Zeitstempel in ms fuer Bildverarbeitungsposition
        Arg3: BallLocation zum Zeitpunkt der Bildverarbeitung
        Arg4: Zeitstempel in ms fuer Fahrtkommando
        Arg5: BallLocation zum Zeitpunkt des Fahrtkommandos */
    void write (unsigned long int, unsigned long int, const BallLocation&, unsigned long int, const BallLocation&) throw ();
  private:
    std::ostream& dest;
    bool binary_mode;
  };


  /** inverse Klasse zu BallLocationWriter */
  class BallLocationReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    BallLocationReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem der naechste BallLocation vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel des Schreibens
        Arg2: Zeitstempel Bildzeitpunkt (Rueckgabe)
        Arg3: letzter gelesener BallLocation Bildzeitpunkt (Rueckgabe)
        Arg4: Zeitstempel Fahrtzeitpunkt (Rueckgabe)
        Arg5: letzter gelesener BallLocation Fahrtzeitpunkt (Rueckgabe)
        Arg6: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, unsigned long int&, BallLocation&, unsigned long int&, BallLocation&, unsigned long int) throw ();
  private:
    std::istream& src;
    unsigned long int next;
    unsigned int encoding_type;
  };

}


#endif
