#include "Socket.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <errno.h> 
#include <iostream>

typedef void raw_type;


using namespace std;

/* --- Helper functions --- */
namespace SimUtils {

static void fillAddr(const string   &address, 
		     unsigned short  port, 
		     sockaddr_in    &addr) 
{
  memset(&addr, 0, sizeof(addr));  // Zero out address structure
  addr.sin_family = AF_INET;       // Internet address
  
  hostent *host;  // Resolve name
  if ((host = gethostbyname(address.c_str())) == NULL) {
    // strerror() will not work for gethostbyname() and hstrerror() 
    // is supposedly obsolete
    throw SocketException("Failed to resolve name (gethostbyname())");
  }
  addr.sin_addr.s_addr = *((unsigned long *) host->h_addr_list[0]);
  
  addr.sin_port = htons(port);     // Assign port in network byte order
}
}
/* ------------------------ */


/* --- SocketException code implementation --- */

SimUtils::SocketException::SocketException(const string &message, bool inclSysMsg) throw() 
  : userMessage(message) 
{
  if (inclSysMsg) {
    userMessage.append(": ");
    userMessage.append(strerror(errno));
  }
}

SimUtils::SocketException::~SocketException() throw() 
{
  ;
}

const char* SimUtils::SocketException::what() const throw() 
{
  return userMessage.c_str();
}

/* ---------------------------------------------*/

/* --- Socket base class code implementation --- */
SimUtils::Socket::Socket(int type, int protocol) throw(SocketException) 
{
  // Make a new socket
  if ((sockDesc = socket(PF_INET, type, protocol)) < 0) {
    throw SocketException("Socket creation failed (socket())", true);
  }
}

SimUtils::Socket::Socket(int sockDesc) {
  this->sockDesc = sockDesc;
}

SimUtils::Socket::~Socket() 
{
  ::close(sockDesc);
  sockDesc = -1;
  std::cerr << "Closed Socket!\n";
}

string SimUtils::Socket::getLocalAddress() throw(SimUtils::SocketException) 
{
  sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);
  
  if (getsockname(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
    throw SocketException("Fetch of local address failed (getsockname())", true);
  }
  return inet_ntoa(addr.sin_addr);
}

unsigned short SimUtils::Socket::getLocalPort() throw(SimUtils::SocketException) 
{
  sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);
  
  if (getsockname(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
    throw SocketException("Fetch of local port failed (getsockname())", true);
  }
  return ntohs(addr.sin_port);
}

void SimUtils::Socket::setLocalPort(unsigned short localPort) throw(SimUtils::SocketException) 
{
  // Bind the socket to its port
  sockaddr_in localAddr;
  memset(&localAddr, 0, sizeof(localAddr));
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  localAddr.sin_port = htons(localPort);
  
  if (bind(sockDesc, (sockaddr *) &localAddr, sizeof(sockaddr_in)) < 0) {
    throw SocketException("Set of local port failed (bind())", true);
  }
}

void SimUtils::Socket::setLocalAddressAndPort(const string &localAddress,
					   unsigned short localPort) throw(SimUtils::SocketException) 
{
  // Get the address of the requested host
  sockaddr_in localAddr;
  fillAddr(localAddress, localPort, localAddr);
  
  if (bind(sockDesc, (sockaddr *) &localAddr, sizeof(sockaddr_in)) < 0) {
    throw SocketException("Set of local address and port failed (bind())", true);
  }
}

unsigned short SimUtils::Socket::resolveService(const string &service,
					     const string &protocol) 
{
  struct servent *serv;        /* Structure containing service information */
  
  if ((serv = getservbyname(service.c_str(), protocol.c_str())) == NULL)
    return atoi(service.c_str());  /* Service is port number */
  else 
    return ntohs(serv->s_port);    /* Found port (network byte order) by name */
}
/* ----------------------------------------------*/


/* --- CommunicatingSocket code implementation --- */
SimUtils::CommunicatingSocket::CommunicatingSocket(int type, int protocol)  throw(SimUtils::SocketException) 
  : Socket(type, protocol) 
{
  ;
}

SimUtils::CommunicatingSocket::CommunicatingSocket(int newSockDesc) 
  : Socket(newSockDesc) 
{
  ;
}

void SimUtils::CommunicatingSocket::connect(const string &foreignAddress,
					 unsigned short foreignPort) throw(SimUtils::SocketException) 
{
  // Get the address of the requested host
  sockaddr_in destAddr;
  fillAddr(foreignAddress, foreignPort, destAddr);
  
  // Try to connect to the given port
  if (::connect(sockDesc, (sockaddr *) &destAddr, sizeof(destAddr)) < 0) {
    throw SocketException("Connect failed (connect())", true);
  }
}

void SimUtils::CommunicatingSocket::send(const void *buffer, 
				      int bufferLen) throw(SimUtils::SocketException) 
{
  if (::send(sockDesc, (raw_type *) buffer, bufferLen, 0) <= 0) {
    throw SocketException("Send failed (send())", true);
  }
}

int SimUtils::CommunicatingSocket::recv(void *buffer, int bufferLen) throw(SimUtils::SocketException) 
{
  int rtn;
  if ((rtn = ::recv(sockDesc, (raw_type *) buffer, bufferLen, 0)) <= 0) {
    throw SocketException("Received failed (recv())", true);
  }
  
  return rtn;
}

string SimUtils::CommunicatingSocket::getForeignAddress() throw(SimUtils::SocketException) 
{
  sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);
  
  if (getpeername(sockDesc, (sockaddr *) &addr,(socklen_t *) &addr_len) < 0) {
    throw SocketException("Fetch of foreign address failed (getpeername())", true);
  }
  return inet_ntoa(addr.sin_addr);
}

unsigned short SimUtils::CommunicatingSocket::getForeignPort() throw(SimUtils::SocketException) 
{
  sockaddr_in addr;
  unsigned int addr_len = sizeof(addr);
  
  if (getpeername(sockDesc, (sockaddr *) &addr, (socklen_t *) &addr_len) < 0) {
    throw SocketException("Fetch of foreign port failed (getpeername())", true);
  }
  return ntohs(addr.sin_port);
}
/* ----------------------------------------------- */


/* --- TCPSocket code implementation --- */
SimUtils::TCPSocket::TCPSocket() throw(SimUtils::SocketException) 
  : CommunicatingSocket(SOCK_STREAM, 
			IPPROTO_TCP) 
{
  ;
}

SimUtils::TCPSocket::TCPSocket(const string &foreignAddress, 
			    unsigned short foreignPort) throw(SimUtils::SocketException) 
  : CommunicatingSocket(SOCK_STREAM, 
			IPPROTO_TCP) 
{
  connect(foreignAddress, foreignPort);
}

SimUtils::TCPSocket::TCPSocket(int newSocketDesc) 
  : CommunicatingSocket(newSocketDesc) 
{
  ;
}
/* ------------------------------------- */


/* --- TCPServerSocket code implementation --- */

SimUtils::TCPServerSocket::TCPServerSocket(unsigned short localPort, 
					int queueLen) throw(SimUtils::SocketException) 
  : Socket(SOCK_STREAM, 
	   IPPROTO_TCP) 
{
  setLocalPort(localPort);
  setListen(queueLen);
}

SimUtils::TCPServerSocket::TCPServerSocket(const string &localAddress, 
					unsigned short localPort, 
					int queueLen) throw(SimUtils::SocketException) 
  : Socket(SOCK_STREAM, 
	   IPPROTO_TCP) 
{
  setLocalAddressAndPort(localAddress, localPort);
  setListen(queueLen);
}

SimUtils::TCPSocket* SimUtils::TCPServerSocket::accept() throw(SimUtils::SocketException) 
{
  int newConnSD;
  if ((newConnSD = ::accept(sockDesc, NULL, 0)) < 0) {
    throw SocketException("Accept failed (accept())", true);
  }
  
  return new TCPSocket(newConnSD);
}

void SimUtils::TCPServerSocket::setListen(int queueLen) throw(SimUtils::SocketException) 
{
  if (listen(sockDesc, queueLen) < 0) {
    throw SocketException("Set listening socket failed (listen())", true);
  }
}
/* ------------------------------------------- */

/* --- UDPSocket code implementation --- */
SimUtils::UDPSocket::UDPSocket() throw(SimUtils::SocketException) 
  : CommunicatingSocket(SOCK_DGRAM,
			IPPROTO_UDP) 
{
  setBroadcast();
}

SimUtils::UDPSocket::UDPSocket(unsigned short localPort)  throw(SimUtils::SocketException) 
  : CommunicatingSocket(SOCK_DGRAM, 
			IPPROTO_UDP) 
{
  setLocalPort(localPort);
  setBroadcast();
}

SimUtils::UDPSocket::UDPSocket(const string &localAddress, 
			    unsigned short localPort) throw(SocketException) 
  : CommunicatingSocket(SOCK_DGRAM, 
			IPPROTO_UDP) 
{
  setLocalAddressAndPort(localAddress, localPort);
  setBroadcast();
}

void SimUtils::UDPSocket::setBroadcast() 
{
  // If this fails, we'll hear about it when we try to send.  This will allow 
  // system that cannot broadcast to continue if they don't plan to broadcast
  int broadcastPermission = 1;
  setsockopt(sockDesc, SOL_SOCKET, SO_BROADCAST, 
             (raw_type *) &broadcastPermission, sizeof(broadcastPermission));
}

void SimUtils::UDPSocket::disconnect() throw(SimUtils::SocketException) 
{
  sockaddr_in nullAddr;
  memset(&nullAddr, 0, sizeof(nullAddr));
  nullAddr.sin_family = AF_UNSPEC;
  
  // Try to disconnect
  if (::connect(sockDesc, (sockaddr *) &nullAddr, sizeof(nullAddr)) < 0) {
    if (errno != EAFNOSUPPORT) {
      throw SocketException("Disconnect failed (connect())", true);
    }
  }
}

void SimUtils::UDPSocket::sendTo(const void *buffer, 
			      int bufferLen, 
			      const string &foreignAddress, 
			      unsigned short foreignPort) throw(SocketException) 
{
  sockaddr_in destAddr;
  fillAddr(foreignAddress, foreignPort, destAddr);
  
  // Write out the whole buffer as a single message.
  if (sendto(sockDesc, (raw_type *) buffer, bufferLen, 0,
             (sockaddr *) &destAddr, sizeof(destAddr)) != bufferLen) {
    throw SocketException("Send failed (sendto())", true);
  }
}

int SimUtils::UDPSocket::recvFrom(void *buffer, 
			       int bufferLen, 
			       string &sourceAddress,
			       unsigned short &sourcePort) throw(SocketException) 
{
  sockaddr_in clntAddr;
  socklen_t addrLen = sizeof(clntAddr);
  int rtn;
  if ((rtn = recvfrom(sockDesc, (raw_type *) buffer, bufferLen, 0, 
                      (sockaddr *) &clntAddr, (socklen_t *) &addrLen)) < 0) {
    throw SocketException("Send failed (sendto())", true);
  }
  sourceAddress = inet_ntoa(clntAddr.sin_addr);
  sourcePort = ntohs(clntAddr.sin_port);

  return rtn;
}

void SimUtils::UDPSocket::setMulticastTTL(unsigned char multicastTTL) throw(SimUtils::SocketException) 
{
  if (setsockopt(sockDesc, IPPROTO_IP, IP_MULTICAST_TTL, 
                 (raw_type *) &multicastTTL, sizeof(multicastTTL)) < 0) {
    throw SocketException("Multicast TTL set failed (setsockopt())", true);
  }
}

void SimUtils::UDPSocket::joinGroup(const string &multicastGroup) throw(SimUtils::SocketException) 
{
  struct ip_mreq multicastRequest;

  multicastRequest.imr_multiaddr.s_addr = inet_addr(multicastGroup.c_str());
  multicastRequest.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(sockDesc, IPPROTO_IP, IP_ADD_MEMBERSHIP, 
                 (raw_type *) &multicastRequest, 
                 sizeof(multicastRequest)) < 0) {
    throw SocketException("Multicast group join failed (setsockopt())", true);
  }
}

void SimUtils::UDPSocket::leaveGroup(const string &multicastGroup) throw(SimUtils::SocketException) {
  struct ip_mreq multicastRequest;

  multicastRequest.imr_multiaddr.s_addr = inet_addr(multicastGroup.c_str());
  multicastRequest.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(sockDesc, IPPROTO_IP, IP_DROP_MEMBERSHIP, 
                 (raw_type *) &multicastRequest, 
                 sizeof(multicastRequest)) < 0) {
    throw SocketException("Multicast group leave failed (setsockopt())", true);
  }
}
/* ------------------------------------- */
