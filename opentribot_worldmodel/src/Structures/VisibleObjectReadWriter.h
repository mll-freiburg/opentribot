
#ifndef Tribots_VisibleObjectReadWriter_h
#define Tribots_VisibleObjectReadWriter_h

#include <iostream>
#include "VisibleObject.h"

namespace Tribots {

  /** Klasse, um VisibleObject und VisibleObjectList zu schreiben */
  class VisibleObjectWriter {
  public:
    /** Konstruktor; Arg1=Zielstream, Arg2=binaeres Format? */
    VisibleObjectWriter (std::ostream&, bool =true) throw ();
    /** Destruktor, fuehrt flush aus */
    ~VisibleObjectWriter () throw ();
    /** schreibe ein VisibleObject;
        Arg1: Zeitstempel in ms, zu der VisibleObject vorlag
        Arg2: Zeitstempel in ms, zu der Bild aufgenommen wurde
        Arg3: erkanntes VisibleObject 
        Arg4: Bildquellen-ID */
    void write (unsigned long int, unsigned long int, const VisibleObject&, unsigned int) throw ();
    /** schreibe eine VisibleObjectList;
        Arg1: Zeitstempel in ms, zu der VisibleObject vorlag
        Arg2: Zeitstempel in ms, zu der Bild aufgenommen wurde
        Arg3: erkannte VisibleObjectList; Zeitstempel der VisibleObjectList wird ignoriert 
        Arg4: Bildquellen-ID */
    void write (unsigned long int, unsigned long int, const VisibleObjectList&, unsigned int) throw ();
  private:
    std::ostream& dest;
    bool binary_mode;
    unsigned long int old_tav;
    unsigned long int old_trec;
  };


  /** inverse Klasse zu VisibleObjectWriter */
  class VisibleObjectReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    VisibleObjectReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem das naechste VisibleObject vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel in ms des Vorliegens des VisibleObject (Rueckgabe)
        Arg2: Zeitstempel in ms des Kamerabildes (Rueckgabe)
        Arg3: Liste der erkannten Objekte sortiert nach Bildquellen (Rueckgabe)
        Arg4: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen. 
        Massgeblich ist der Zeitpunkt des Vorliegens der Information
        Return: wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, unsigned long int&, std::vector<VisibleObjectList>&, unsigned long int) throw ();
  private:
    std::istream& src;
    unsigned long int next;
    unsigned int encoding_type;  // 0=ascii, 1=binaer

    // fuer binaeres Lesen:
    unsigned long int old_tav;
    unsigned long int old_trec;
    VisibleObject::ObjectType old_objecttype;
    unsigned int old_id;
    unsigned long int binary_read_old ();
  };

}


#endif
