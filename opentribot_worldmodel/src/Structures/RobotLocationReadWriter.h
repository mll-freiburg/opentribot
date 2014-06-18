
#ifndef Tribots_RobotLocationReadWriter_h
#define Tribots_RobotLocationReadWriter_h

#include <iostream>
#include "RobotLocation.h"

namespace Tribots {

  /** Klasse, um RobotLocation zu schreiben */
  class RobotLocationWriter {
  public:
    /** Konstruktor; Arg1=Zielstream, Arg2=binaeres Format? */
    RobotLocationWriter (std::ostream&, bool = true) throw ();
    /** Destruktor, fuehrt flush aus */
    ~RobotLocationWriter () throw ();
    /** schreibe ein RobotLocation;
        Arg1: Zeitstempel in ms des schreibens
        Arg2: Zeitstempel in ms fuer Bildverarbeitungsposition
        Arg3: RobotLocation zum Zeitpunkt der Bildverarbeitung
        Arg4: Zeitstempel in ms fuer Fahrtkommando
        Arg5: RobotLocation zum Zeitpunkt des Fahrtkommandos */
    void write (unsigned long int, unsigned long int, const RobotLocation&, unsigned long int, const RobotLocation&) throw ();
  private:
    std::ostream& dest;
    bool binary_mode;
  };


  /** inverse Klasse zu RobotLocationWriter */
  class RobotLocationReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    RobotLocationReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem der naechste RobotLocation vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel des Schreibens
        Arg2: Zeitstempel Bildzeitpunkt (Rueckgabe)
        Arg3: letzter gelesener RobotLocation Bildzeitpunkt (Rueckgabe)
        Arg4: Zeitstempel Fahrtzeitpunkt (Rueckgabe)
        Arg5: letzter gelesener RobotLocation Fahrtzeitpunkt (Rueckgabe)
        Arg6: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, unsigned long int&, RobotLocation&, unsigned long int&, RobotLocation&, unsigned long int) throw ();
  private:
    std::istream& src;
    unsigned long int next;
    unsigned int encoding_type;
  };

}


#endif
