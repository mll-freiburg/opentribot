
#ifndef _Tribots_UDPCommunicationStatistics_h_
#define _Tribots_UDPCommunicationStatistics_h_

#include <netinet/in.h>
#include "../Fundamental/Time.h"

namespace Tribots {

  /** Struktur, um Informationen ueber ein ein- oder ausgehendes Paket zu speichern */
  struct UDPPacketStatistics {
    unsigned int size;  ///< Paketgroesse in Byte
    Time timestamp;  ///< Zeitpunkt des Sendens/Empfangens (send/receive-Kommandos)
    struct sockaddr_in partner_address;  ///< Adresse des Empfaengers/Absenders
  };

  /** Struktur, um Informationen ueber den Kommunikations-Verkehr zu speichern */
  struct UDPCommunicationStatistics {
    double packet_rate;  ///< mittlere Anzahl gesendeter/empfangener Pakete pro Sekunde
    double packet_size;  ///< mittlere Packetgroesse der zuletzt gesendeten/empfangenen Pakete in Byte
  };

}

#endif
