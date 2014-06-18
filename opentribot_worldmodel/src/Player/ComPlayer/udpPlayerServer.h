#ifndef _udp_player_server_h_
#define _udp_player_server_h_

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>

#include "../../Fundamental/Time.h"
#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/RobotData.h"
#include "../../Structures/DriveVector.h"

#define BUF_MAX_LEN 4096

class udpPlayerServer{
 public:
  socklen_t count;
  struct sockaddr_in MyPort, RemoteSocket;
  char send_buf[BUF_MAX_LEN];
  int  send_buf_len;
  char recv_buf[BUF_MAX_LEN];
  int  recv_buf_len;

  int BindSocket, ConnectionSocket;

  bool put(const std::stringstream &sbuf);

 public:
  udpPlayerServer();
  ~udpPlayerServer();
  bool init(int port);

  bool wait_for_client();

  int receive();
  bool receive_all(int timeout_ms);
  bool send();

  bool getInit();
  
  bool putTime(Tribots::Time t);
  bool getTime(Tribots::Time &t);

  bool getDriveVector(Tribots::DriveVector &dv);

  bool putRobotLocation(Tribots::RobotLocation rl);
  bool putBallLocation(Tribots::BallLocation bl);
  bool putRobotData(Tribots::RobotData rd);
  bool putGameState(int gs);
};

#endif
