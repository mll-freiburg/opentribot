
#ifndef _Tribots_GyroDataReadWriter_h_
#define _Tribots_GyroDataReadWriter_h_

#include <iostream>
#include "GyroData.h"

namespace Tribots {

  /** Klasse, um GyroData zu schreiben */
  class GyroDataWriter {
  public:
    /** Konstruktor; Arg=Zielstream */
    GyroDataWriter (std::ostream&) throw ();
    /** Destruktor, fuehrt flush aus */
    ~GyroDataWriter () throw ();
    /** schreibe ein GyroData;
        Arg1: Zeitstempel in ms des Schreibens
        Arg2: Zeitstempel in ms fuer Erfassung
        Arg3: Gyroskop-Messung */
    void write (unsigned long int, unsigned long int, const GyroData&) throw ();
  private:
    std::ostream& dest;
  };


  /** inverse Klasse zu GyroDataWriter */
  class GyroDataReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    GyroDataReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem der naechste GyroData vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel des Schreibens
        Arg3: Zeitstempel Erfassung (Rueckgabe)
        Arg3: letzter gelesener GyroDatensatz (Rueckgabe)
        Arg4: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, unsigned long int&, GyroData&, unsigned long int) throw ();
  private:
    std::istream& src;
    unsigned long int next;
  };

}


#endif
