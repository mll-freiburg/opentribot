
#ifndef Tribots_UDPSocket_h
#define Tribots_UDPSocket_h

#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>

namespace Tribots {

  /** Einfache UDP-Verbindung; kann sowohl als Server als auch als Client verwendet werden */
  class UDPSocket {
  public:
    UDPSocket () throw ();
    virtual ~UDPSocket () throw ();
    /** als Server an Port arg1 initialisieren; Rueckgabewert: erfolgreich? */
    virtual bool init_as_server (int) throw ();
    /** als Client initialisieren; Verbindung mit Rechner arg1 an Port arg2; Rueckgabewert: erfolgreich? */
    virtual bool init_as_client (const char*, int) throw ();
    /** Verbindung schliessen */
    virtual void close () throw ();
    /** Daten senden an den Kommunikationspartner, mit dem zuletzt kommuniziert wurde (als Sender oder Empfaenger)
        Arg1: Sendepuffer,
        Arg2: Groesse des Sendepuffers in Byte,
        Return: true bei Erfolg, sonst false,
        Beachte: UDP-Protokoll erfolgreiches Versenden != erfolgreiches Empfangen */
    virtual bool send (const char*, unsigned int) throw ();
    /** Daten senden;
        Arg1: Empgaengeradresse
        Arg2: Sendepuffer,
        Arg3: Groesse des Sendepuffers in Byte,
        Return: true bei Erfolg, sonst false,
        Beachte: UDP-Protokoll erfolgreiches Versenden != erfolgreiches Empfangen */
    virtual bool sendto (const struct sockaddr_in&, const char*, unsigned int) throw ();
    /** Daten lesen, falls vorhanden;
        Arg1: Rueckgabepuffer fuer empfangene Daten (Speicher muss bereits allokiert sein),
        Arg2: Rueckgabewert, Menge der empfangenen Daten in Byte,
        Arg3: Argument, Groesse des Puffers Arg1 in Byte,
        Return: etwas gelesen?
        Beachte: liegen mehr Daten vor als in den puffer passen, wird nach Arg3 Byte abgeschnitten */
    virtual bool receive (char*, unsigned int&, unsigned int) throw ();
    /** Adresse des letzten Kommunikationspartners */
    virtual const struct sockaddr_in& partner_address () const throw ();
    /** true, wenn init_as_client oder init_as_server erfolgreich aufgerufen wurde */
    virtual bool started() const throw ();
    /** true, wenn die letzte Transaktion erfolgreich war, d.h. es ist kein Fehler aufgetreten */
    virtual bool okay() const throw ();
    /** liefert true, wenn als Server gestartet */
    virtual bool as_server() const throw ();
    int socketDescriptor;

  protected:
    bool is_connected;
    bool is_okay;
    bool is_server;

    struct sockaddr_in partnerAddress;
  };

}

bool operator== (const struct sockaddr_in&, const struct sockaddr_in&) throw ();

#endif
