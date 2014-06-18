
#ifndef Tribots_MessageBoardReadWriter_h
#define Tribots_MessageBoardReadWriter_h

#include <iostream>
#include "MessageBoard.h"

namespace Tribots {

  /** Klasse, um MessageBoard zu schreiben */
  class MessageBoardWriter {
  public:
    /** Konstruktor; Arg1=Zielstream, Arg2=binaeres Format? Arg2 wird z.Zt mangels Implementierung einer ASCII-Variante ignoriert */
    MessageBoardWriter (std::ostream&, bool = true) throw ();
    /** Destruktor, fuehrt flush aus */
    ~MessageBoardWriter () throw ();
    /** schreibe ein MessageBoard;
        Arg1: Zeitstempel in ms des schreibens
        Arg2: MessageBoard */
    void write (unsigned long int, const MessageBoard&) throw ();
    /** schreibe ein MessageBoard;
        Arg1: Zeitstempel in ms des schreibens
        Arg2: Ausgehende Nachrichten
        Arg3: Eingehende Nachrichten */
    void write (unsigned long int, const std::vector<std::string>&, const std::vector<std::string>&) throw ();
  private:
    std::ostream& dest;
    bool binary_mode;
    std::vector<std::string> oldOutgoing;
    std::vector<std::string> oldIncoming;
    void matchMessages (char delChar, char insChar, std::vector<std::string>& oldList, const std::vector<std::string>& newList);
    std::vector<std::string> removeDuplicates (const std::vector<std::string>& list);
  };


  /** inverse Klasse zu MessageBoardWriter */
  class MessageBoardReader {
  public:
    /** Konstruktor; Arg=Quellstream */
    MessageBoardReader (std::istream&) throw ();
    /** liefert in Arg1 den Zeitstempel in ms, zu dem der naechste MessageBoard vorlag;
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool next_timestamp (unsigned long int&) const throw ();
    /** lese alle abgespeicherten Objekte mit Zeitstempel bis zu einem Zeitpunkt (arg4);
        Arg1: Zeitstempel des Schreibens
        Arg2: letzte Liste eingehender Nachrichten (Rueckgabe)
        Arg3: letzte Liste ausgehender Nachrichten (Rueckgabe)
        Arg4: Zeitstempel. bis zu dem Nachrichten gelesen werden sollen
        Return: true, wenn ueberhaupt ein naechstes Objekt vorliegt, false, wenn Ende
        der Datei erreicht wurde */
    bool read_until (unsigned long int&, std::vector<std::string>&, std::vector<std::string>&, unsigned long int) throw (std::bad_alloc);
  private:
    std::istream& src;
    unsigned long int next;
    unsigned int encoding_type;
    std::vector<std::string> oldOutgoing;
    std::vector<std::string> oldIncoming;
  };

}


#endif
