
#include <iostream>
#include <cstdlib>
#include "../../Communication/PriorityUDPCommunication.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<2) {
    cerr << argv[0] << " Port\n";
    return -1;
  }
  int port = atoi (argv[1]);
  PriorityUDPCommunication sock ("xy");
  sock.init_as_server (port);

  while (true) {
    unsigned int success = sock.receive ();
    cout << "Received " << success << " packets ";
    const char* buffer;
    unsigned int len;
    bool okay = sock.get (buffer, len);
    while (okay) {
      cout << '[';
      for (unsigned int j=0; j<len; j++)
	cout << buffer[j];
      cout << "] ";
      okay = sock.get (buffer, len);
    }
    cout << '\n';

    sleep (3);
  }

  sock.close();
  return 0;
}
