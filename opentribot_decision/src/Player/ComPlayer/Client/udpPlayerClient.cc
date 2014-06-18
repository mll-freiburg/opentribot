#include "udpPlayerClient.h"
#include <sys/time.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netdb.h>

udpPlayerClient::udpPlayerClient()
{
  ;
}

udpPlayerClient::~udpPlayerClient()
{
  if (ConnectionSocket >= 0) close(ConnectionSocket);
}

bool udpPlayerClient::init( int port, const char * addr)
{
  ConnectionSocket = socket ( AF_INET, SOCK_DGRAM, 0);
  fcntl(ConnectionSocket, F_SETFL, O_NONBLOCK);

  if( ConnectionSocket <= 0 )
    {
      std::cerr << "Can't bind socket ... \n";
      return false;
    }
  
  if ( inet_aton( addr , &destination.sin_addr ) < 0)
    {
      std::cerr << "Can't init server address!\n";
      return false;
    }
  destination.sin_family = AF_INET;
  destination.sin_port = htons( port );
  
  bzero(&(destination.sin_zero), 8);
 
  return true;
}


bool udpPlayerClient:: connect_to_server()
{
  bool res=false;
  int i=0;
  
  while (i<10 && !res)
    {
      putInit();
      send();

      res=receive_all(1000);
      i++;
    }

  res&=getInit();

  return res;
}


int udpPlayerClient::receive()
{
  char * buf_ptr = &recv_buf[recv_buf_len];
  int space_left = BUF_MAX_LEN - recv_buf_len;

  if (space_left <= 1)
    {
      std::cerr << "Buffer can't hold more data, may be it should be cleared?? \n";
      return -1;
    }

  socklen_t addr_len = sizeof(destination);
  int numBytesRead = recvfrom(ConnectionSocket, buf_ptr, BUF_MAX_LEN, 0,
			      (struct sockaddr *)&destination, &addr_len);
  if (numBytesRead >= 0)
    {
      recv_buf_len += numBytesRead;
    }
  return numBytesRead;
}

bool udpPlayerClient::receive_all(int timeout_ms)
{
  recv_buf_len = 0;
  
  fd_set readfds;
  
  FD_ZERO(&readfds);
  FD_SET(ConnectionSocket, &readfds);

  if (timeout_ms>0)
    {
      struct timeval tv;
      tv.tv_sec  = timeout_ms / 1000;
      tv.tv_usec = (timeout_ms - (tv.tv_sec *1000)) * 1000;
      select(ConnectionSocket+1, &readfds, NULL, NULL, &tv);
    }
  else
    {
      select(ConnectionSocket+1, &readfds, NULL, NULL, NULL);
    }

  if (FD_ISSET(ConnectionSocket, &readfds))
    {
      while (receive() > 0);
      recv_buf[recv_buf_len]='\0';
      //std::cout << "Received: [" << recv_buf << "]\n"; 
    }
  else
    {
      std::cerr << "Nothing received after " << timeout_ms << "\n";
      return false;
    }

  return true;  
}

bool udpPlayerClient::send()
{
  int numbytes_send = sendto(ConnectionSocket, send_buf, send_buf_len,
			     0, (struct sockaddr *)&destination, sizeof(struct sockaddr));
  if (numbytes_send < 0 ) 
    {
      std::cerr << "Problems n sending ... \n";
      return false;
    }
  // std::cout << "Sended:[" << send_buf << "]\n";
  send_buf_len=0;
  return true;
}

bool udpPlayerClient::put(const std::stringstream &sbuf)
{
  int sbuf_len = strlen(sbuf.str().c_str());
  
  if ((sbuf_len + send_buf_len) > BUF_MAX_LEN)
    {
      std::cerr << "Can't put more Data in send_buf\n\r";
      return false;
    }

  char * buf_ptr = &send_buf[send_buf_len];
  
  memcpy(buf_ptr, sbuf.str().c_str(), sbuf_len);
  
  send_buf_len+= sbuf_len;
  
  //std::cout << "Put :{" << sbuf.str().c_str() << "} in buffer\n\r" << std::flush;

  return true;
}

bool udpPlayerClient::putInit()
{
  std::stringstream sbuf;
  sbuf << "@INIT ";
  return put(sbuf);
}

bool udpPlayerClient::getInit()
{
  if (recv_buf_len == 0) return false;

  char *ptr= strstr(recv_buf, "@INIT");
  if (ptr != NULL)
    return true;
  else
    return false;
}

bool udpPlayerClient::getGameState(int &gs)
{
  char *ptr= strstr(recv_buf, "@GAMESTATE");
  if (ptr != NULL)
    {
      int _gs;
      int res=sscanf(ptr, "@Gamestate[ %d ]", &_gs);
      if (res < 1) return false;
      else
	{
	  gs=_gs;
	  return true;
	}
    }
  return false;
}

bool udpPlayerClient::getRobotLocation(double &x, double &y, double &phi, 
				       double &vx, double &vy, double &vphi, 
				       double &qual, bool &kicking)
{
  char *ptr= strstr(recv_buf, "@ROBOTLOCATION");
  if (ptr == NULL) return false;
  
  double _x, _y, _phi;
  double _vx, _vy, _vphi, _qual;
  int    _ikick;

  int res=sscanf(ptr, "@ROBOTLOCATION[ POS( %lf , %lf , %lf ); VEL( %lf , %lf , %lf ); QUAL( %lf ); KICK( %d );]",
		 &_x , &_y , &_phi, &_vx, &_vy, &_vphi, &_qual, &_ikick);

  if (res < 8) {
    std::cerr << "Problems on parsing: {" << ptr << "}\n\r";
    return false;
  }

  x=_x; y=_y; phi=_phi;
  vx=_vx; vy=_vy; vphi=_vphi;
  qual=_qual;
  kicking = (_ikick!=0);
  return true;
}


bool udpPlayerClient::getBallLocation(double &x, double &y, double &vx, double &vy,  double &qual, bool &hold)
{
  char *ptr= strstr(recv_buf, "@BALLLOCATION");
  if (ptr == NULL) return false;
  
  double _x, _y;
  double _vx, _vy, _qual;
  int    _ihold;

  int res=sscanf(ptr, "@BALLLOCATION[ POS( %lf , %lf ); VEL( %lf , %lf ); QUAL( %lf ); HOLDING( %d ); ]",
		 &_x , &_y , &_vx, &_vy, &_qual, &_ihold);

  if (res < 6) {
    std::cerr << "Problems on parsing: {" << ptr << "}\n\r";
    return false;
  }

  x=_x; y=_y;
  vx=_vx; vy=_vy;
  qual=_qual;
  hold = (_ihold!=0);
  return true;
}


bool udpPlayerClient::getRobotData_WHEELVEL(double &v1, double &v2, double &v3)
{
  double _v1, _v2, _v3;

  char *ptr= strstr(recv_buf, "@ROBOTDATA");
  if (ptr == NULL)
    {
      std::cout << "Can't find @ROBOTDATA in {" << ptr << "}\n";
      return false;
    }
  ptr= strstr(ptr, "WHEELVEL");
  if (ptr == NULL) 
    {
      std::cout << "Can't find WHEELVEL in {" << ptr << "}\n";
      return false;
    }
  int res=sscanf(ptr,"WHEELVEL( %lf , %lf , %lf );",&_v1, &_v2, &_v3);
  if (res < 3) {
    std::cout << "Problems on parsing {" << ptr << "}\n";
    return false;
  }
  v1=_v1;
  v2=_v2;
  v3=_v3;
  return true;
}

bool udpPlayerClient::putTime(long int t_ms)
{
  std::stringstream sbuf;
  sbuf << "@TIME[" << t_ms << "]";
  return put(sbuf);
}

bool udpPlayerClient::getTime(long int &_t_ms)
{
  char *ptr= strstr(recv_buf, "@TIME");
  if (ptr != NULL)
    {
      long int t_ms;
      int res = sscanf(ptr,"@TIME[%ld]",&t_ms);
      if (res < 1) {
	std::cerr << "Problems on parsing {" << ptr << "}\n\r";
	return false;
      }
      _t_ms = t_ms;
      return true;
    }
  return false;
}


bool udpPlayerClient::putDriveVector(double vx, double vy, double vphi, bool kick)
{
  std::stringstream sbuf;
  sbuf << "!DRIVEVECTOR[ "
       << "VEL( "
       << vx << " , "
       << vy << " , "
       << vphi << " ); "
       << "KICK( "
       << kick << " ); "
       << " ]  ";
  return put(sbuf);
}

#if 1
int main ( int argc, char **argv)
{
  udpPlayerClient client;

  client.init(7010,"127.0.0.1");

  if (client.connect_to_server()) {
  while (1)
    {
      client.receive_all();
      long int t_ms;
      client.getTime(t_ms);


      client.putDriveVector(0,0,2,0);
      client.putTime(t_ms);
      client.send();
    }
  }
  else
    std::cerr << "Kann keinen Server finden!\n";
}


#endif
