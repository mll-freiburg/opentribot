
#include <iostream>
#include <cstdlib>
#include <sstream>
#include "../../Communication/PriorityUDPCommunication.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " Host Port\n";
    return -1;
  }
  int port = atoi (argv[2]);
  PriorityUDPCommunication sock ("xy", 514);
  sock.init_as_client (argv[1], port);

  char buffer [500];
  unsigned int len=0;

  unsigned long int li=0;
  while (true) {
    /*
    ostringstream os;
    os << li;
    string s = os.str();
    s.length()+1;
    for (unsigned int j=0; j<20; j++) {
      buffer[0]='a'+j;
      for (unsigned int i=0; i+1<len; i++) {
	buffer[i+1]=s[i];
	buffer[len]='\0';
      }
      std::cerr << "putting [" << buffer << "]\n";
      sock.put (buffer, len, j/10);
    }
    */
    ostringstream os;
    os << "123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890";
    string s = os.str();
    sock.put (s.c_str(), s.length());

    
    unsigned int success = sock.send ();
    std::cerr << "sending " << success << " packets\n";
    
    usleep (400000);
    li++;
  }
  
  sock.close();
  return 0;
}
