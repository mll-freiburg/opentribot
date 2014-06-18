
#include <iostream>
#include <cstdlib>
#include "../../Communication/TaggedUDPCommunication.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<2) {
    cerr << argv[0] << " Port\n";
    return -1;
  }
  int port = atoi (argv[1]);
  TaggedUDPCommunication sock ("xy");
  sock.init_as_server (port);

  while (true) {
    unsigned int success = sock.receive ();
    cout << "Received " << success << " packets\n";
    const char* buffer;
    unsigned int len;
    for (unsigned char tag=0; tag<5; tag++) {
      cout << "Tag #" << (int)(tag) << ": ";
      if (sock.get (tag, buffer, len)) {
	for (unsigned int j=0; j<len; j++)
	  cout << buffer[j];
      } else
	cout << "---";
      cout << '\n';
    }

    sleep (3);
  }

  sock.close();
  return 0;
}
