//test unison
#include "odesimcomm.h"
#include <string.h>
//#include "macro_msg.h"
#include <sys/time.h>
#include <unistd.h>



int
ClientData::add_client (sockaddr_in client_addr)
{
  for (int i = 0; i < MAX_CLIENTS; i++)
    {
      if (!client_active[i])
	{
	  serv_addr[i] = client_addr;
	  client_active[i] = true;
	  client_not_responding[i] = 0;
	  return i;		// return number of added client
	}


    }
  return -1;			// list full no more clients available
}


int
ClientData::is_active_client (sockaddr_in client_addr)
{
  for (int i = 0; i < MAX_CLIENTS; i++)
    {
      if (serv_addr[i].sin_addr.s_addr == client_addr.sin_addr.s_addr &&
	  serv_addr[i].sin_port == client_addr.sin_port &&
	  client_active[i] == true)
	{
	  client_not_responding[i] = 0;
	  return i;		// return number of client
	}
    }

  //not found as active client
  return -1;
}


void
ClientData::check_for_inactivity ()
{

  for (int i = 0; i < MAX_CLIENTS; i++)
    {
      client_not_responding[i] = client_not_responding[i] + 1;
      if (client_active[i] == true && client_not_responding[i] > 30)
	{
	  client_active[i] = false;
	  cout << "Client " << i << " not Responding" << endl;
	}
    }



}


/****************************************************************************/
/****************************************************************************/
OdeSimComm::~OdeSimComm ()
{
}

bool
OdeSimComm::use_as_server (int port)
{
  is_server = true;		// if you need to know inside some functions whether this is a server
  return sock.init_socket_fd (port);
}

bool
OdeSimComm::use_as_client (char const *addr, int port)
{
  is_server=false;
  sock.init_socket_fd (0);
  return sock.init_serv_addr (addr, port);
}

bool
OdeSimComm::use_nonblock ()
{
  sock.set_fd_nonblock ();
  return true;
}




bool
OdeSimComm::send (std::string msg)
{
  sock.send_msg ((char *) msg.c_str (), msg.length ());
  return true;
}




bool
OdeSimComm::receive (char *msg)
{
  int length;
  if (sock.recv_msg ((char *) msg, length, 1500))
    {

      if (this->is_server)
	{

	  /*   for (int i=0;i<MAX_CLIENTS;i++)
	     {
	     cout<<"ACTIVE CLIENT "<< i << " : " <<clientdata.client_active[i]<<endl;
	     }

	   */
	  // cerr <<"Receiving from INET "<<sock.serv_addr.sin_addr.s_addr<<"  PORT "<<sock.serv_addr.sin_port<<endl;

	  last_client = this->clientdata.is_active_client (sock.serv_addr);
	  //cout <<"communication with client no :"<<last_client<<endl;

	  if (last_client == -1)
	    {
	      int res = this->clientdata.add_client (sock.serv_addr);
	      cout << "NEW CLIENT CONNECTED :" << res << endl;
	      last_client = res;
	      num_clients++;
	    }
	}

      return true;

    }
  else
    {
      return false;
    }

}




bool
OdeSimComm::update_name (std::string msg)
{
  clientdata.names[last_client] = msg;
  return true;
}

bool
OdeSimComm::update_team (std::string msg)
{
  clientdata.teams[last_client] = msg;
  return true;
}
