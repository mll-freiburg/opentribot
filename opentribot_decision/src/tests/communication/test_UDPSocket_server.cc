
#include <iostream>
#include <cstdlib>
#include "../../Communication/UDPSocket.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<2) {
    cerr << argv[0] << " port\n";
    return -1;
  }
  int port = atoi (argv[1]);
  UDPSocket sock;
  sock.init_as_server (port);

  char buffer [201];

  while (true) {
    unsigned int len;
    if (sock.receive (buffer, len, 200)) {
      buffer[len]='\0';
      std::cerr << "receiving message [" << buffer << "]\n";
      for (unsigned int i=0; i<len; i++)
	buffer[i] = toupper (buffer[i]);      
      std::cerr << "sending message [" << buffer << "]\n";
      sock.send (buffer, len);
    }
    std::cerr << "Started=" << sock.started() << "; Okay=" << sock.okay() << '\n';
    sleep (1);
  }

  sock.close();
  return 0;
}
