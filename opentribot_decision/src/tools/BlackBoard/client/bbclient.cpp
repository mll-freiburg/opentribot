#include "bbclient.h"


int
BBClient::init (string  hostname)
{
  bbcomm = new BBComm ();
  bbcomm->use_as_server (0);
  bbcomm->use_as_client (hostname.c_str (), 30001);
  bbcomm->use_nonblock ();
  bbcomm->send ("/NAME Tribot1 /TEAM Tribots");


}


int
BBClient::update ()
{



  int packets = 0;
  int retval;
  string msg;
  char buffer[2000];

  while ((retval = bbcomm->receive (buffer)))
    {
      msg.assign (buffer);
      cout << "received " << msg << endl;
      packets++;
      timeval ts1;
      gettimeofday (&ts1, NULL);


      if (packets == 0)
	{
	  cerr << "SimClient: Received nothing." << endl;
	}
/*      else if (packets > 1)
	{
	  cerr << "SimClient: Received " << packets -
	    1 << " old packets." << endl;
	}
*/


    }

}

