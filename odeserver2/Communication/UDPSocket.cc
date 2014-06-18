
#include "UDPSocket.h"
#include <iostream>
#include <cstring>

using namespace Tribots;
using namespace std;


UDPSocket::UDPSocket () throw () : is_connected(false), is_okay(false) {;}

UDPSocket::~UDPSocket () throw () {
  if (is_connected)
    ::close (socketDescriptor);
}

bool UDPSocket::init_as_server (int port) throw () {
  if (is_connected)
    ::close(socketDescriptor);

  // Socket erzeugen
  socketDescriptor = socket (AF_INET, SOCK_DGRAM, 0);
  if (socketDescriptor<0) {
    cerr << "cannot create socket\n";
    return false;
  }

  // Socket an Port binden
  partnerAddress.sin_family = AF_INET;
  partnerAddress.sin_addr.s_addr = htonl(INADDR_ANY);
  partnerAddress.sin_port = htons(port);
  if (bind(socketDescriptor,
           (struct sockaddr *) &partnerAddress,
           sizeof(partnerAddress)) < 0) {
    cerr << "cannot bind socket";
    ::close (socketDescriptor);
    return false;
  }

  is_server=true;
  is_connected=true;
  is_okay=true;
  return true;
}


bool UDPSocket::init_as_client (const char* hostname, int port) throw () {
  if (is_connected)
    ::close(socketDescriptor);

  // Host bestimmen
  struct hostent* hostInfo = gethostbyname(hostname);
  if (hostInfo == NULL) {
    cout << "problem interpreting host: " << hostname << "\n";
    return false;
  }

  // Socket erzeugen
  socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
  if (socketDescriptor < 0) {
    cerr << "cannot create socket\n";
    return false;
  }
  
  partnerAddress.sin_family = hostInfo->h_addrtype;
  memcpy((char *) &partnerAddress.sin_addr.s_addr,
         hostInfo->h_addr_list[0], hostInfo->h_length);
  partnerAddress.sin_port = htons(port);
  
  is_server=false;
  is_connected=true;
  is_okay=true;
  return true;
}


void UDPSocket::close () throw () {
  if (is_connected)
    ::close(socketDescriptor);
  is_connected=is_okay=false;
}


bool UDPSocket::sendto (const struct sockaddr_in& add, const char* buffer, unsigned int buflen) throw () {
  partnerAddress = add;
  return send (buffer, buflen);
}


bool UDPSocket::send (const char* buffer, unsigned int buflen) throw () {
  if (!is_connected)
    return false;
  int buflen2=static_cast<int>(buflen);
  is_okay = (::sendto(socketDescriptor, buffer, static_cast<int>(buflen2), 0,
		 (struct sockaddr *) &partnerAddress,
		 sizeof(partnerAddress)) == buflen2);
  return is_okay;
}


bool UDPSocket::receive (char* buffer, unsigned int& buflen, unsigned int maxbuflen) throw () {
  if (!is_connected) {
    buflen=0;
    return false;
  }
  fd_set readSet;
  timeval timeout;
  struct sockaddr_in senderAddress;
  socklen_t senderAddressLength = sizeof(senderAddress);
  
  FD_ZERO(&readSet);
  FD_SET(socketDescriptor, &readSet);
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
  if (select(socketDescriptor+1, &readSet, NULL, NULL, &timeout)) {
    int buflen2 = recvfrom(socketDescriptor, buffer, maxbuflen, 0,
		      (struct sockaddr *) &senderAddress,
		      &senderAddressLength);

    if (buflen2<0) {
      // Kommunikation schief gelaufen
      is_okay=false;
      return false;
    }
    buflen=buflen2;

    if (is_server && (senderAddress.sin_addr.s_addr!=partnerAddress.sin_addr.s_addr || senderAddress.sin_port!=partnerAddress.sin_port)) {
      // wenn partnerAddress nicht die Adresse ist, von der empfange wurde, dann umstellen
      //      std::cerr << "redirecting partner address from port " << ntohs(partnerAddress.sin_port) << " to port " << ntohs(senderAddress.sin_port) << '\n';
      partnerAddress.sin_addr.s_addr = senderAddress.sin_addr.s_addr;
      partnerAddress.sin_port = senderAddress.sin_port;
    }

    // Kommunikation erfolgreich
    is_okay=true;
    return true;
  }

  // es lagen keine Nachrichten vor
  return false;
}


const struct sockaddr_in& UDPSocket::partner_address () const throw () {
  return partnerAddress;
}


bool UDPSocket::started() const throw () {
  return is_connected;
}


bool  UDPSocket::okay() const throw () {
  return is_okay;
}


bool UDPSocket::as_server() const throw () {
  return is_connected && is_server;
}


bool operator== (const struct sockaddr_in& a1, const struct sockaddr_in& a2) throw () {
  if (a1.sin_port!=a2.sin_port) return false;
  if (a1.sin_addr.s_addr!=a2.sin_addr.s_addr) return false;
  return true;
}

