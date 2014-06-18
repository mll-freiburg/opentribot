
#ifndef Tribots_NonspecificTaggedUDPCommunication_h
#define Tribots_NonspecificTaggedUDPCommunication_h

#include "TaggedUDPCommunication.h"
#include <string>

namespace Tribots {

  /** TaggedUDPCommunication mit ein paar Hilfsfunktionen
      zum effizienten Codieren von Informationen, aber ohne
      put() und get()-Routinen; can in abgeleiteten Klassen
      individuell hinzugefuegt werden */
  class NonspecificTaggedUDPCommunication {
  public:
    /** Konstruktor, Arg1: zwei Markierungsbytes, Arg2: Maximalgroesse einer Nachricht in Byte */
    NonspecificTaggedUDPCommunication (const char*, unsigned int =8189) throw (std::bad_alloc);
    virtual ~NonspecificTaggedUDPCommunication () throw ();
    /** als Server an Port arg1 initialisieren; Rueckgabewert: erfolgreich? */
    virtual bool init_as_server (int) throw ();
    /** als Client initialisieren; Verbindung mit Rechner arg1 an Port arg2; Rueckgabewert: erfolgreich? */
    virtual bool init_as_client (const char*, int) throw ();
    /** Verbindung schliessen */
    virtual void close () throw ();
    /** Wie lange in Byte darf eine Nachricht maximal sein? */
    virtual unsigned int max_message_length () const throw ();
    /** Adresse des letzten Kommunikationspartners */
    virtual const struct sockaddr_in& partner_address () const throw ();
    /** true, wenn init_as_client oder init_as_server erfolgreich aufgerufen wurde */
    virtual bool started() const throw ();
    /** liefert true, wenn als Server gestartet */
    virtual bool as_server() const throw ();

    /** Alle Pakete senden;
        Return: Anzahl gesendeter Pakete */
    virtual unsigned int send () throw ();
    /** Alle Pakete an bestimmten Empfaenger (Arg1) senden;
        Return: Anzahl gesendeter Pakete */
    virtual unsigned int sendto (const struct sockaddr_in&) throw ();
    /** Pakete des naechsten Senders empfangen;
        Return: Anzahl fehlerfrei empfangener Pakete */
    virtual unsigned int receive () throw (std::bad_alloc);
    /** Pakete eines Senders (Arg1) empfangen;
        Return: Anzahl fehlerfrei empfangener Pakete */
    virtual unsigned int receivefrom (const struct sockaddr_in&) throw (std::bad_alloc);
    /** den Ausgabepuffer loeschen; wird nach "send" auch automatisch aufgerufen */
    virtual void clear_send_buffer () throw ();
    /** den Eingabepuffer loeschen; wird vor "receive" auch automatisch aufgerufen */
    virtual void clear_receive_buffer () throw ();

    /** berechnet die durchschnittliche Kommunikationsbelastung durch ausgehende Pakete in den letzten maximal (arg1) Sekunden */
    virtual UDPCommunicationStatistics send_load (int =10) const throw ();
    /** berechnet die durchschnittliche Kommunikationsbelastung durch eingehende Pakete in den letzten maximal (arg1) Sekunden */
    virtual UDPCommunicationStatistics receive_load (int =10) const throw ();

  protected:
    TaggedUDPCommunication socket;
  
  public:
    // Hilfsfunktionen zum lesen/schreiben von numerischen Werten in 2Byte:
    static void write_signed_short (char*, signed short int) throw ();
    static void write_unsigned_short (char*, unsigned short int) throw ();
    static void write_signed_short (char*, double) throw ();
    static void write_unsigned_short (char*, double) throw ();
    static signed short int read_signed_short (const char*) throw ();
    static unsigned short int read_unsigned_short (const char*) throw ();
    static char* write_string_nl (char*, const std::string&, char = '\n') throw ();  ///< schreibt einen string in einen Puffer und anschliessend ein '\n', liefert Zeiger auf ein Element nach '\n'
    static const char* read_string_nl (const char*, std::string&, unsigned int =0, char = '\n') throw (); ///< liest aus einem Puffer einen String bis zu einem abschliessenden '\n'. liefert Zeiger auf Element nach dem '\n', liest maximal (arg3) Zeichen, (0=unendlich)
  };

}

#endif
