#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stuttgartinterface.h"


using namespace std;





int
main (int argc, char **argv)
{
  StuttgartInterface interface;
interface.hostname="laphell";//129.69.216.1";
interface.port=20002;


StuttgartMessage msg;
msg.constructStringMessage("Test");

string option;
if (argc>1)option.append(argv[1]);

if(option=="-test"){
string pcmsg="MSG::PC::1::B::RefBoxCommandTester::System_Player_Commands::I::-1193056328//throwInOpp//::END";
// Typen PC= PlayerCommand  WM =WomoData SM= Stringmessage
// Header MSG::Type::ID::MODUS::SENDERID::MODULE_RECEIVE::IMPORTANCE::TIMESTAMP[::IPADRESS_RECV::HOSTNAME::RECV_ID]//
// BODY  PC: 1 String Refboxmessage
// BODY WOMO Feste Struktur
// BODY SM   einzelnerString
// BODY SL   Comma separierte list
// MODUS B Broadcast A Adressed                            I sofort N normal


string contents = "MSG::WM::1::B::rfcbot1::System_WoMo_Data::N::-1267039682//12,INT,-1267039691,4,1,0,0,0,0,INT,-1267039682,4,1,-6877,-164,52,0,INT,-1267039691,7,1,1,-4358,3874,1568,1649,650,2795,INT,-1267039691,9,0,INT,-1267039718,2,0,INT,-1267039718,6,1,1,-4196,3486,1166,1878,0,INT,-1267039682,3,1,16,-16,-8,INT,-1267039682,7,1,-6877,-164,52,0,-6877,-164,52,INT,-31985088,3,0,INT,0,5,0,INT,-1267039682,3,0,STRING,-1267039682,10,1,wait,0,RefereeEvents,K,O,Stop,0,1,N,0//::END";
strcpy(interface.buf,contents.c_str());
interface.disectmessage();
exit(0);
}
  while (true)
    {
      usleep (300000);
      interface.communicate ();
    }
  return 0;
}
