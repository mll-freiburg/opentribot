#include <cmath>
#include <time.h>


#include "bbcomm.h"
#include <cstdio>
#include <string>
#include <sstream>
#include <iostream>
#include <sstream>
#define N 20			// number of robots

using namespace std;

// dynamics and collision objects

int simloopCount = 0;

static BBComm serverobject;



using namespace std;



int
parse (string msg)
{

  string item;
  stringstream ss;
  ss.str (msg);

  //while (getline(ss,item,'/')){
  while (ss >> item)
    {
      if (item == "/NAME")
	{
	  ss >> item;
	  cout << "Name ist : " << item << endl;
	  serverobject.update_name (item);
	}
      if (item == "/TEAM")
	{
	  ss >> item;
	  cout << "Team ist : " << item << endl;
	  serverobject.update_team (item);

	}
    }




}

int translate(string& from,string& to,string &msg){
string item;
string returnstring;
  stringstream ss;
  ss.str (msg);

cout << "Translating"<<endl;



while (ss>> item){
if (item=="FUSSBALL"){
item="SOCCERBALL";
}
if (item=="POS"){
int posx,posy;
ss>> posx;
ss>> posy;
ostringstream os;
os <<"POS "<<(posy)<<" "<<(-posx);
//cout <<"OSTRINSTREAM "<<os.str()<<endl;
item=os.str();
}


returnstring.append(item+" ");

}




cout <<"Returning "<<returnstring<<endl;

}





static void
saveStatesToDisk ()
{
  FILE *outfile;
  outfile = fopen ("savestate.bin", "w");
//fwrite(&savesimstate,sizeof(savesimstate),3,outfile);
  fclose (outfile);
  cout << "saving" << endl;



}


static void
loadStatesFromDisk ()
{
  FILE *infile;
  infile = fopen ("savestate.bin", "r");
//fread(&savesimstate,sizeof(savesimstate),3,infile);
  fclose (infile);
  cout << "Falls kein Segmentation fault = >Loading ready" << endl;



}


static void
communicate ()
{

  // receive and try to map the client to a robbi and send back appropriate Worldmodel data
  int howmanyreceived;
  howmanyreceived = 0;
  serverobject.clientdata.check_for_inactivity ();

  char buffer[1000];
  string msg;
  while (serverobject.receive (buffer))
    {				// Receive all client data
      msg.assign (buffer);
//      cout << "Receiving Data: " << msg.c_str () << endl;
      cout << serverobject.clientdata.names[serverobject.
								last_client]
	<<": "<<msg<< endl;
      parse (msg);
      string te,na;
      te.assign("TRIBOTS");
      na.assign("RFC");

      translate(te,na,msg);

      for (int i = 0; i < serverobject.num_clients; i++)
	{
	  if (i != serverobject.last_client)
	    {
	      serverobject.sock.serv_addr =
		serverobject.clientdata.serv_addr[i];
	      cout << "sending To CLient" << endl;
	      serverobject.send (msg);
	    }
	}


      howmanyreceived++;

    }
  //cout <<"Number of Received Infos: "<<howmanyreceived<<endl;



  for (int i = 0; i < N; i++)
    {

      if (serverobject.clientdata.client_active[i])
	{

	  serverobject.sock.serv_addr = serverobject.clientdata.serv_addr[i];
//        serverobject.send ();
	}
      else
	{			// when client inactive, reset the steering


	}



    }








}




// simulation loop 1 player im moment Joystick


static void
command (int cmd)
{

//  cout <<"Taste:"<<cmd<<endl;
//  size_t i;
//  int j,k;

  if (cmd == 'B')
    {
    }


}



int
main ()
{
  serverobject.use_as_server (30001);
  serverobject.use_nonblock ();

  fd_set fdset;
  int stdinfd = fileno (stdin);
  int udpfds = serverobject.sock.socket_fd;
  struct timeval tv;
  struct timeval tv2;


  while (true)
    {
      FD_ZERO (&fdset);
      FD_SET (stdinfd, &fdset);
      FD_SET (udpfds, &fdset);
      int max_fd = serverobject.sock.socket_fd + 1;

      // no timeout
      tv.tv_sec = 0;
      tv.tv_usec = 0;
      // do not block for the msg
      int rc = select (max_fd, &fdset, NULL, NULL, &tv);
      if (rc < 0)
	continue;
      if (FD_ISSET (udpfds, &fdset))
	{
	  // udp fd match
//      memset (incomingbuffer, 0, 4096);
//      serversocket.recv_msg (incomingbuffer, bufferlength);
//      got_UDP_this_frame = true;
	//  cout << " UDP PACKET" << endl;
	  communicate();

	}
      else if (FD_ISSET (stdinfd, &fdset))
	{
	  // keyboard input
//      handleKeyboardInput ();
	  //    goto beforeselect1;
	  cout << "Keyboard INput" << endl;
	}

    }
  return 0;
}



