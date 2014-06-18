
#include "../../Communication/UDPSocket.h"
#include <iostream>
#include <string>
#include "../../Fundamental/RemoteTune.h"

using namespace std;
using namespace Tribots;

int main (int argc, char **argv) {
  RemoteTune* a=RemoteTune::getTheRemoteTune();

  if (argc==1){
    cout <<"Usage : remotetune host [port]"<<endl;
    cout <<"l List variables"<<endl;
    cout <<"n next variable"<<endl;
    cout <<"p previous variable"<<endl;
    cout <<"c double"<<endl;
  }

  if (argc == 2  || argc == 3) {
    // CLIENT MODE
    cout << "RemoteTune Client Mode connecting to "<<argv[1] << endl;

    int port = 9997;
    if (argc==3)
      port = atoi(argv[2]);
    a->initClient (argv[1], port);
    while (true) {
      a->client_process();
      usleep(100000);
    }
  }

  if (argc>3){
    // SERVER MODE, zum Testen
    a->initServer (9997);
    double localvalue=123.0;
    double lv2 =200;
    double testdouble=1.123;
    //int testint=4;
    //bool testbool=true;
    a->registerlv("LVINMAIN",&localvalue);
    a->registerlv("LV2",&lv2);
    a->registerlv("NV",&testdouble);
    // a.registerlv("INteger",&testint);
    // a.registerlv("booltest",&testbool);
    a->registerlv("NV",&testdouble);

    while (true) {
      a->server_process();
      usleep(100000);
    }
  }
}
