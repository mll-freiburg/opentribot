
#ifndef Tribots_MultiPacketUDPCommunication_h
#define Tribots_MultiPacketUDPCommunication_h

#include "UDPSocket.h"
#include "UDPCommunicationStatistics.h"
#include "../Fundamental/RingBuffer.h"
#include <vector>
#include <deque>

namespace Tribots {
  
  /** UDP-Verbindung, die mehrere Pakete auf einmal senden oder empfangen kann;
      unterscheidet verschiedene Absender und sortiert eingehende Nachrichten nach Absendern */
  class MultiPacketUDPCommunication {
  public:
    MultiPacketUDPCommunication () throw ();
    virtual ~MultiPacketUDPCommunication () throw ();
    /** als Server an Port arg1 initialisieren; Rueckgabewert: erfolgreich? */
    virtual bool init_as_server (int) throw ();
    /** als Client initialisieren; Verbindung mit Rechner arg1 an Port arg2; Rueckgabewert: erfolgreich? */
    virtual bool init_as_client (const char*, int) throw ();
    /** Verbindung schliessen */
    virtual void close () throw ();

    /** Mehrere Pakete senden an letzten Kommunikationspartner,
        Arg1: Array mit zu sendenden Paketen
        Arg2: Array mit den Groessen der zu sendenen Pakete in Byte 
        Return: Anzahl erfolgreich gesendeter Pakete */
    virtual unsigned int send (const std::vector<char*>, const std::vector<unsigned int>) throw ();
    /** Mehrere Pakete senden an letzten Kommunikationspartner,
        Arg1: Empfaengeradresse
        Arg2: Array mit zu sendenden Paketen
        Arg3: Array mit den Groessen der zu sendenen Pakete in Byte 
        Return: Anzahl erfolgreich gesendeter Pakete */
    virtual unsigned int sendto (const struct sockaddr_in&, const std::vector<char*>, const std::vector<unsigned int>) throw ();

    /** Alle empfangenen Pakete des ersten Kommunikationspartners liefern,
        Arg1: Rueckgabearray fuer empfangene Puffer (Puffer werden dynamisch erzeugt),
        Arg2: Rueckgabewert, Groesse der Rueckgabepuffer in Byte,
        Arg3: Argument, Maximale Puffergroesse in Byte
        Return: Anzahl erfolgreich empfangener Pakete */
    virtual unsigned int receive (std::vector<char*>&, std::vector<unsigned int>&, unsigned int =8192) throw (std::bad_alloc);
    /** Alle empfangenen Pakete eines Kommunikationspartners liefern,
        Arg1: Senderadresse,
        Arg2: Rueckgabearray fuer empfangene Puffer (Puffer werden dynamisch erzeugt),
        Arg3: Rueckgabewert, Groesse der Rueckgabepuffer in Byte,
        Arg4: Argument, Maximale Puffergroesse in Byte
        Return: Anzahl erfolgreich empfangener Pakete */
    virtual unsigned int receivefrom (const struct sockaddr_in&, std::vector<char*>&, std::vector<unsigned int>&, unsigned int =8192) throw (std::bad_alloc);

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
    UDPSocket socket;
    struct sockaddr_in partnerAddressMulti;
    std::deque<struct sockaddr_in> senderAddresses;
    std::deque<std::vector<char*> > receivedBuffer;
    std::deque<std::vector<unsigned int> > receivedBufferLength;
    RingBuffer<UDPPacketStatistics> receivePacketStatistics;
    RingBuffer<UDPPacketStatistics> sendPacketStatistics;
    virtual void receive_all (unsigned int) throw (std::bad_alloc);
  };

}

#endif
