
#ifndef Tribots_TaggedUDPCommunication_h
#define Tribots_TaggedUDPCommunication_h

#include "PriorityUDPCommunication.h"

namespace Tribots {

  /** UDP-Verbindung, die Nachrichten mit Tags verschickt; als Tags ist 
      1 Byte vorgesehen, so dass maximal 256 verschiedene Nachrichtentypen 
      unterschieden werden koennen; */
  class TaggedUDPCommunication {
  public:
    /** Konstruktor; Arg1: zwei Markierungsbytes, Arg2: Maximalgroesse einer Nachricht in Byte */
    TaggedUDPCommunication (const char*, unsigned int =8189) throw (std::bad_alloc);
    virtual ~TaggedUDPCommunication () throw ();
    /** als Server an Port arg1 initialisieren; Rueckgabewert: erfolgreich? */
    virtual bool init_as_server (int) throw ();
    /** als Client initialisieren; Verbindung mit Rechner arg1 an Port arg2; Rueckgabewert: erfolgreich? */
    virtual bool init_as_client (const char*, int) throw ();
    /** Verbindung schliessen */
    virtual void close () throw ();

    /** Wie lange in Byte darf eine Nachricht maximal sein? */
    virtual unsigned int max_message_length () const throw ();

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

    /** eine Nachricht hinzufuegen;
        Arg1: Nachrichtentag
        Arg2: die Nachricht,
        Arg3: Laenge der Nachricht in Byte,
        Arg4: Prioritaet der Nachricht,
        Return: konnte Nachricht hinzugefuegt werden? */
    virtual bool put (unsigned char, const char*, unsigned int, unsigned int =0) throw (std::bad_alloc);
    /** empfange Nachricht holen
        Arg1: Nachrichtentag
        Arg2: Rueckgabewert: ein Zeiger auf den Puffer,
        Arg3: Rueckgabewert: die Laenge der Nachricht in Byte,
        Arg4: die wievielte unter den empfangenen Bachrichten dieses Schluessels gelesen werden soll (je groesser der Index, desto neuer die Nachricht),
        Return: war Nachricht vorhanden? 
        Liefert false, falls Arg4 groesser als die Anzahl Nachrichten ist */
    virtual bool get (unsigned char, const char*&, unsigned int&, unsigned int) throw ();
    /** letzte empfangene Nachricht holen; wie (get(.,.,.,.)), liefert die letzte Nachricht */
    virtual bool get (unsigned char, const char*&, unsigned int&) throw ();
    /** Anzahl empfangener Nachrichten mit Schluessel Arg1 */
    virtual unsigned int num_messages (unsigned char) throw ();

    /** Adresse des letzten Kommunikationspartners */
    virtual const struct sockaddr_in& partner_address () const throw ();
    /** true, wenn init_as_client oder init_as_server erfolgreich aufgerufen wurde */
    virtual bool started() const throw ();
    /** liefert true, wenn als Server gestartet */
    virtual bool as_server() const throw ();

    /** berechnet die durchschnittliche Kommunikationsbelastung durch ausgehende Pakete in den letzten maximal (arg1) Sekunden */
    virtual UDPCommunicationStatistics send_load (int =10) const throw ();
    /** berechnet die durchschnittliche Kommunikationsbelastung durch eingehende Pakete in den letzten maximal (arg1) Sekunden */
    virtual UDPCommunicationStatistics receive_load (int =10) const throw ();
  
  protected:
    PriorityUDPCommunication socket;
    std::vector<std::vector<const char*> > received_messages;
    std::vector<std::vector<unsigned int> > received_message_length;
    virtual unsigned int receive_intern (const struct sockaddr_in*) throw (std::bad_alloc);
  };

}

#endif
