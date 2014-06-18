

#include "bbclient.h"

int
main (int c, char **argv)
{
  
  BBClient bbc;

  string host;
  if (c == 2)
    host = argv[1];
  else
    {
      cerr << "Please supply hostname for connection" << endl;

      exit (-1);

    }

  bbc.init(host);



  // Send something so the server knows we are he    re and the port and inet adress

  string sendmessage;

  for (int i = 0; i < 100; i++)
    {

      cout << "---:";
      getline (cin, sendmessage);
      bbc.bbcomm->send (sendmessage.c_str ());
      bbc.update ();

    }







}
