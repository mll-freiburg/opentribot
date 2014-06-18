#ifndef _ODESIMCOMMUNICATION_H_
#define _ODESIMCOMMUNICATION_H_

#include "udpsocket.h"
//#include "../macro_msg.h"
//#include "../global_defs.h"
#define MAX_CLIENTS 16
#include <iostream>
using namespace std;



struct ClientData
{
  // Adresses of connected clients
  struct sockaddr_in serv_addr[MAX_CLIENTS];
  string names[MAX_CLIENTS];
  string teams[MAX_CLIENTS];

  bool client_active[MAX_CLIENTS];
  int client_not_responding[MAX_CLIENTS];
    ClientData ()
  {
    for (int i = 0; i < MAX_CLIENTS; i++)
      {
	client_not_responding[i] = 0;
	client_active[i] = false;

      }

  }
  int is_active_client (sockaddr_in client_addr);
  int add_client (sockaddr_in client_addr);
  void check_for_inactivity ();


};



class OdeSimComm
{
  static const int BUFFER_MAX_LEN = 8096;
  int buffer_len;
  char buffer[BUFFER_MAX_LEN];
  bool is_server;
public:
    UDPSocket sock;
  int num_clients;

    OdeSimComm ()
  {
    is_server = false;
    num_clients = 0;
  }

   ~OdeSimComm ();
  bool use_as_server (int port);
  bool use_as_client (char const *addr, int port);

  bool use_nonblock ();


  bool send (std::string msg);
  bool receive (char *msg);

  int last_client;
  ClientData clientdata;
  bool update_name (std::string name);
  bool update_team (std::string name);


};








#endif
