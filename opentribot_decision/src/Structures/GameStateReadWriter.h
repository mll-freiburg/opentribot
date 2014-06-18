
#ifndef _Tribots_GameStateReadWriter_h_
#define _Tribots_GameStateReadWriter_h_

#include <iostream>
#include "GameState.h"

namespace Tribots {

  /** Klasse, um GameState zu schreiben */
  class GameStateWriter {
  public:
    /** Konstruktor; Arg1=Zielstream, Arg2=binaeres Format? */
    GameStateWriter (std::ostream&, bool = true) throw ();
    /** Destruktor, fuehrt flush aus */
    ~GameStateWriter () throw ();
    /** schreibe ein GameState;
        Arg1: Zeitstempel in ms
        Arg2: eigentlicher GameState
        Arg3: Spielertyp
        Arg4: Spielerrolle
        Arg5: Behavior */
    void write (unsigned long int, const GameState&, const char* ptype, const char* prole, const char* pbehavior) throw ();
  private:
    std::ostream& dest;
    std::string playertype;
    std::string playerrole;
    std::string behavior;
    bool binary_mode;
  };


  /** inverse Klasse zu GameStateWriter */
  class GameStateReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    GameStateReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem der naechste GameState vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel (Rueckgabe)
        Arg2: letzter gelesener GameState (Rueckgabe)
        Arg3: Spielertyp (Rueckgabe)
        Arg4: Spielerrolle (Rueckgabe)
        Arg5: Behavior (Rueckgabe)
        Arg6: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, GameState&, std::string& ptype, std::string& prole, std::string& pbehavior, unsigned long int) throw ();
  private:
    std::istream& src;
    unsigned long int next;
    std::string playertype;
    std::string playerrole;
    std::string behavior;
    unsigned int encoding_type;
  };

}


#endif
