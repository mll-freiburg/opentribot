
#include <iostream>
#include <cstdlib>
#include <sstream>
#include "../../Communication/TaggedUDPCommunication.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " Host Port\n";
    return -1;
  }
  int port = atoi (argv[2]);
  TaggedUDPCommunication sock ("xy");
  sock.init_as_client (argv[1], port);

  char buffer [500];
  unsigned int len=0;

  unsigned long int li=0;
  while (true) {
    ostringstream os;
    os << li;
    string s = os.str();
    len = s.length()+1;
    buffer[0]='a';
    for (unsigned int i=0; i+1<len; i++)
      buffer[i+1]=s[i];
    buffer[len]='\0';
    std::cerr << "putting [" << buffer << "]\n";
    sock.put (li%5, buffer, len);
    
    unsigned int success = sock.send ();
    std::cerr << "sending " << success << " packets\n";
    
    sleep (1);
    li++;
  }
  
  sock.close();
  return 0;
}
