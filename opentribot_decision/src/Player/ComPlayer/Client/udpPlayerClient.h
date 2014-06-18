#ifndef _udp_player_client_h_
#define _udp_player_client_h_

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <sstream>

#define BUF_MAX_LEN 4096

class udpPlayerClient{
 public:
  socklen_t count;
  struct sockaddr_in destination;
  char send_buf[BUF_MAX_LEN];
  int  send_buf_len;
  char recv_buf[BUF_MAX_LEN];
  int  recv_buf_len;

  int ConnectionSocket;

  bool put(const std::stringstream &sbuf);

 public:
  udpPlayerClient();
  ~udpPlayerClient();
  bool init(int port, const char * addr);
  bool connect_to_server();

  bool send();
  int receive();
  bool receive_all(int timeout_ms = -1);

  bool putInit();
  bool getInit();
  
  bool putTime(long int t_ms);
  bool getTime(long int &t_ms);

  bool putDriveVector(double vx, double vy, double vphi, bool kick);
  
  bool getGameState(int &gs);
  bool getRobotLocation(double &x, double &y, double &phi, double &vx, double &vy, double &vphi, double &qual, bool &kicking);
  bool getBallLocation(double &x, double &y, double &vx, double &vy,  double &qual, bool &hold);
  bool getRobotData_WHEELVEL(double &v1, double &v2, double &v3);
  

};

#endif
