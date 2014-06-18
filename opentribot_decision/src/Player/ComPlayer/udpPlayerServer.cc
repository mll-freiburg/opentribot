#include "udpPlayerServer.h"
#include <sys/time.h>
#include <fcntl.h>
#include <string.h>
#include <sstream>

udpPlayerServer::udpPlayerServer()
{
  ;
}

udpPlayerServer::~udpPlayerServer()
{
  if (ConnectionSocket >= 0) close(ConnectionSocket);
}

bool udpPlayerServer::init( int port)
{
  send_buf_len = 0;
  recv_buf_len = 0;
  ConnectionSocket = socket ( AF_INET, SOCK_DGRAM, 0);
  fcntl(ConnectionSocket, F_SETFL, O_NONBLOCK);
  if( ConnectionSocket <= 0 )
    {
      std::cerr << "Can't bind socket ... \n";
      return false;
    }

  MyPort.sin_port = htons( port );
  MyPort.sin_family = AF_INET;
  MyPort.sin_addr.s_addr = INADDR_ANY;
 
  bzero(&(MyPort.sin_zero), 8);

  if ( bind(ConnectionSocket, (struct sockaddr *)&MyPort, sizeof(MyPort))<0)
    {
      std::cerr << "Can't bind socket ... \n";
      return false;
    }
  
  return true;
}


bool udpPlayerServer::wait_for_client()
{
  int i=0;
  bool res=false;
  while (i<10 && !res)
    {
      res=receive_all(1000);
      if (!res)
	std::cout << "Waiting for client! ... \n\r";
      i++;
    }
  if (!res) return res;

  if (!getInit())
    {
      std::cout << "Something received but not the expected thing ... \n\r";
      return false;
    }

  sprintf(send_buf, "@INIT");
  send_buf_len = strlen(send_buf);
  res&=send();

  usleep(1000); // gib client Zeit sich zu initialisieren 

  return res;
}


int udpPlayerServer::receive()
{
  char * buf_ptr = &recv_buf[recv_buf_len];
  int space_left = BUF_MAX_LEN - recv_buf_len;

  if (space_left <= 1)
    {
      std::cerr << "Buffer can't hold more data, may be it should be cleared?? \n\r";
      return -1;
    }

  socklen_t addr_len = sizeof(RemoteSocket);
  int numBytesRead = recvfrom(ConnectionSocket, buf_ptr, BUF_MAX_LEN, 0,
			      (struct sockaddr *)&RemoteSocket, &addr_len);
  if (numBytesRead >= 0) {
    recv_buf_len += numBytesRead;
  }
  return numBytesRead;
}

bool udpPlayerServer::receive_all(int timeout_ms)
{
  recv_buf_len = 0;
  recv_buf[0] = '\0';

  struct timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms - (tv.tv_sec *1000)) * 1000;

  fd_set readfds;

  FD_ZERO(&readfds);
  FD_SET(ConnectionSocket, &readfds);

  select(ConnectionSocket+1, &readfds, NULL, NULL, &tv);

  if (FD_ISSET(ConnectionSocket, &readfds))
    {
      while (receive() > 0);
      recv_buf[recv_buf_len]='\0';
      std::cout << "Received: [" << recv_buf << "]\n\r"; 
    }
  else
    {
      std::cerr << "Nothing received after " << timeout_ms << "\n\r";
      return false;
    }
  return true;  
}


bool udpPlayerServer::send()
{
  send_buf[send_buf_len] = '\0';
  int numbytes_send = sendto(ConnectionSocket, send_buf, send_buf_len,
			     0, (struct sockaddr *)&RemoteSocket, sizeof(struct sockaddr));
  if (numbytes_send < 0 ) 
    {
      std::cerr << "Problems n sending ... \n\r";
      return false;
    }
  //std::cout << "Sended:[" << send_buf << "] len: " << send_buf_len <<  "\n\r";
  send_buf_len=0;
  send_buf[0]='\0';
  return true;
}

bool udpPlayerServer::getInit()
{
  if (recv_buf_len == 0) return false;

  char *ptr= strstr(recv_buf, "@INIT");
  if (ptr != NULL)
    return true;
  else
    return false;
}

bool udpPlayerServer::getTime(Tribots::Time &t)
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
      t.set_msec(t_ms);
      return true;
    }
  return false;
}


bool udpPlayerServer::put(const std::stringstream &sbuf)
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

bool udpPlayerServer::putTime(Tribots::Time t)
{
  std::stringstream sbuf;
  
  sbuf << "@TIME[ " << t.get_msec() << " ]  ";

  return put(sbuf);
}

bool udpPlayerServer::putRobotLocation(Tribots::RobotLocation rl)
{
  std::stringstream sbuf;
  sbuf << "@ROBOTLOCATION[ " 
       << "POS( "
       << rl.pos.x << " , " 
       << rl.pos.y << " , "
       << rl.heading.get_rad() << "); "
       << "VEL( "
       << rl.vtrans.x << " , "
       << rl.vtrans.y << " , "
       << rl.vrot
       << "); "
       << "QUAL( "
       << rl.quality 
       << "); "
       << "KICK( "
       << rl.kick
       << "); "
       << " ]  ";
  return put(sbuf);
}

bool udpPlayerServer::putBallLocation(Tribots::BallLocation bl)
{
  std::stringstream sbuf;
  sbuf << "@BALLLOCATION[ " 
       << "POS( "
       << bl.pos.x << " , " 
       << bl.pos.y << " ); "
       << "VEL( "
       << bl.velocity.x << " , "
       << bl.velocity.y << " ); "
       << "QUAL( "
       << bl.quality << "); "
       << " ]  ";
  return put(sbuf);
}

bool udpPlayerServer::putRobotData(Tribots::RobotData rd)
{
  std::stringstream sbuf;
  sbuf << "@ROBOTDATA[ "
       << "ID( "
       << rd.robotIdString << " , "
       << rd.BoardID << " ); "
       << "MOTORSON( "
       << rd.motors_on << " ); "
       << "WHEELVEL( "
       << rd.wheel_vel[0] << " , "
       << rd.wheel_vel[1] << " , "
       << rd.wheel_vel[2] << " ); "
       << "ROBOTVEL( "
       << rd.robot_vel[0] << " , "
       << rd.robot_vel[1] << " , "
       << rd.robot_vel[2] << " ); "
       << "MOTORCURRENT( "
       << rd.motor_current[0] <<  " , "
       << rd.motor_current[1] <<  " , "
       << rd.motor_current[2] <<  " ); "
       << "MOTOROUTPUT( "
       << rd.motor_output[0] <<  " , "
       << rd.motor_output[1] <<  " , "
       << rd.motor_output[2] <<  " ); "
       << "MOTORTEMPSWITCH( "
       << rd.motor_temp_switch[0] << " , "
       << rd.motor_temp_switch[1] << " , "
       << rd.motor_temp_switch[2] << " ); "
       << "MOTORTEMP( "
       << rd.motor_temp[0] << " , "
       << rd.motor_temp[1] << " , "
       << rd.motor_temp[2] << " ); "
       << "MOTORVCC( "
       << rd.motor_vcc
       << " ); "
       << " ]  ";
  return put(sbuf);
}

bool udpPlayerServer::putGameState(int gs)
{
  std::stringstream sbuf;
  sbuf << "@GAMESTATE[ "
       << gs << " ]  ";
  return put(sbuf);
}

bool udpPlayerServer::getDriveVector(Tribots::DriveVector &dv)
{
  char *ptr= strstr(recv_buf, "!DRIVEVECTOR");
  if (ptr != NULL)
    {
      double vx, vy, vphi;
      int ikick;
      int res = sscanf(ptr,"!DRIVEVECTOR[ VEL( %lf , %lf , %lf ); KICK( %d ); ]",
		       &vx, &vy, &vphi, &ikick);
      if (res < 4) {
	std::cerr << "Problems on parsing DV from {" << ptr << "}\n\r";
	return false;
      }
      
      dv.vtrans.x = vx;
      dv.vtrans.y = vy;
      dv.vrot     = vphi;
      dv.kick     = (ikick != 0);
      return true;
    }
  return false;
}

#if 0
int main ( int argc, char **argv)
{
  udpPlayerServer serv;

  serv.init(7010);

  if (serv.wait_for_client()) {

  timeval tv1, tv2;

  while (1) {

    gettimeofday(&tv1,0);

    sprintf(serv.send_buf, "HALLO CLIENT!\n");
    serv.send_buf_len = strlen(serv.send_buf);
    serv.send();

    serv.receive_all(500);

    gettimeofday(&tv2,0);

    float diff = ((tv2.tv_sec-tv1.tv_sec)*1000
      + (tv2.tv_usec-tv1.tv_usec)/1000.0);

    std::cout << "Time for a com: " << diff << " ms\n";
    
  }
  }
  else
    std::cerr << "Kein Client connected!\n";

}


#endif
