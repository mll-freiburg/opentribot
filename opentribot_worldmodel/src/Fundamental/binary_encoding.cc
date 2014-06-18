
#include "binary_encoding.h"

void Tribots::wunsigned3 (std::ostream& os, unsigned long int src) throw () {
  os.put((src>>16)&0xff);
  os.put((src>>8)&0xff);
  os.put(src&0xff);
}
  
void Tribots::wunsigned2 (std::ostream& os, unsigned long int src) throw () {
  os.put((src>>8)&0xff);
  os.put(src&0xff);
}

void Tribots::wunsigned1 (std::ostream& os, unsigned long int src) throw () {
  os.put(src&0xff);
}



unsigned long int Tribots::runsigned3 (std::istream& is) throw () {
  char c1, c2, c3;
  is.get (c1);
  is.get (c2);
  is.get (c3);
  return (static_cast<unsigned long int>(static_cast<unsigned char>(c1))<<16)|
      (static_cast<unsigned long int>(static_cast<unsigned char>(c2))<<8)|
      (static_cast<unsigned long int>(static_cast<unsigned char>(c3)));
}

unsigned long int Tribots::runsigned2 (std::istream& is) throw () {
  char c1, c2;
  is.get (c1);
  is.get (c2);
  return (static_cast<unsigned long int>(static_cast<unsigned char>(c1))<<8)|
      (static_cast<unsigned long int>(static_cast<unsigned char>(c2)));
}

unsigned long int Tribots::runsigned1 (std::istream& is) throw () {
  char c1;
  is.get (c1);
  return (static_cast<unsigned long int>(static_cast<unsigned char>(c1)));
}




// Testroutinen:
#if 0

#include <sstream>

using namespace std;
using namespace Tribots;

int main () {
  cout << "unsigned3\n";
  for (unsigned long int i=0; i<=16000000ul; i+=1000ul) {
    stringstream inout;
    wunsigned3 (inout, i);
    inout << '\n';
    unsigned long int res = runsigned3 (inout);
    if (i!=res) {
      cout << i << ' ' << res << '\n';
    }
  }
  cout << "unsigned2\n";
  for (unsigned long int i=0; i<=65000ul; i+=1000ul) {
    stringstream inout;
    wunsigned2 (inout, i);
    inout << '\n';
    unsigned long int res = runsigned2 (inout);
    if (i!=res) {
      cout << i << ' ' << res << '\n';
    }
  }
  cout << "unsigned1\n";
  for (unsigned long int i=0; i<=255ul; i+=1ul) {
    stringstream inout;
    wunsigned1 (inout, i);
    inout << '\n';
    unsigned long int res = runsigned1 (inout);
    if (i!=res) {
      cout << i << ' ' << res << '\n';
    }
  }
  return 0;
}

#endif
