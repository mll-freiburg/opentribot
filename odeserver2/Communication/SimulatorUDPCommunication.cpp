
#include "SimulatorUDPCommunication.h"
#include <cstring>

using namespace TribotsSim;
using namespace std;

// Tags sind wiefolgt belegt (alle mit Prioritaet 0)
//
// 0: Ping
// 10: Robot
// 20: Ball
// 30: DriveVector
// 40: Obstacles
// 254: Bye


namespace {
  // um diese beiden Variablen nicht in jeder einzelnen Methode erneut deklarieren zu muessen, hier:
  const char* retbuf;
  unsigned int retlen;
  const char magic_bytes [] = { -23, 12 };
}


SimulatorUDPCommunication::SimulatorUDPCommunication () throw (std::bad_alloc) : NonspecificTaggedUDPCommunication (magic_bytes) {;}

SimulatorUDPCommunication::~SimulatorUDPCommunication () throw () {;}

void SimulatorUDPCommunication::close () throw () {
  clear_send_buffer();
  putBye ();
  send ();
  NonspecificTaggedUDPCommunication::close ();
}

void SimulatorUDPCommunication::free_socket () throw () {
  NonspecificTaggedUDPCommunication::close ();
}

bool SimulatorUDPCommunication::putPing () throw (std::bad_alloc) {
  return socket.put (0,NULL,0);
}
bool SimulatorUDPCommunication::getPing () throw () {
  return socket.get (0,retbuf,retlen);
}
bool SimulatorUDPCommunication::putBye () throw (std::bad_alloc) {
  return socket.put (254,NULL,0);
}
bool SimulatorUDPCommunication::getBye () throw () {
  return socket.get (254,retbuf,retlen);
}
bool SimulatorUDPCommunication::putRobot (double x, double y, double phi, double vx, double vy, double vphi, bool kick) throw (std::bad_alloc) {
  char buffer [13];
  write_signed_short (buffer, x);
  write_signed_short (buffer+2, y);
  write_signed_short (buffer+4, 100.0*phi);
  write_signed_short (buffer+6, 1000*vx);
  write_signed_short (buffer+8, 1000*vy);
  write_signed_short (buffer+10, 1000*vphi);
  buffer [12]=(kick ? 1 : 0);
  return socket.put (10,buffer,13);
}
bool SimulatorUDPCommunication::getRobot (double& x, double& y, double& phi, double& vx, double& vy, double& vphi, bool& kick) throw () {
  if (socket.get (10,retbuf,retlen) && retlen>=13) {
    x=read_signed_short (retbuf);
    y=read_signed_short (retbuf+2);
    phi=read_signed_short (retbuf+4)/100.0;
    vx=0.001*read_signed_short (retbuf+6);
    vy=0.001*read_signed_short (retbuf+8);
    vphi=0.001*read_signed_short (retbuf+10);
    kick=(retbuf+12!=0);
    return true;
  }
  return false;
}
bool SimulatorUDPCommunication::putBall (double x, double y, double z, double vx, double vy, double vz) throw (std::bad_alloc) {
  char buffer [12];
  write_signed_short (buffer, x);
  write_signed_short (buffer+2, y);
  write_signed_short (buffer+4, z);
  write_signed_short (buffer+6, 1000*vx);
  write_signed_short (buffer+8, 1000*vy);
  write_signed_short (buffer+10, 1000*vz);
  return socket.put (20,buffer,12);
}
bool SimulatorUDPCommunication::getBall (double& x, double& y, double& z, double& vx, double& vy, double& vz) throw () {
  if (socket.get (20,retbuf,retlen) && retlen>=12) {
    x=read_signed_short (retbuf);
    y=read_signed_short (retbuf+2);
    z=read_signed_short (retbuf+4);
    vx=0.001*read_signed_short (retbuf+6);
    vy=0.001*read_signed_short (retbuf+8);
    vz=0.001*read_signed_short (retbuf+10);
    return true;
  }
  return false;
}
bool SimulatorUDPCommunication::putDriveVector (double vx, double vy, double vphi, unsigned int klen) throw (std::bad_alloc) {
  char buffer [7];
  write_signed_short (buffer, 1000*vx);
  write_signed_short (buffer+2, 1000*vy);
  write_signed_short (buffer+4, 1000*vphi);
  buffer[6]=klen;
  return socket.put (30,buffer,7);
}
bool SimulatorUDPCommunication::getDriveVector (double& vx, double& vy, double& vphi, unsigned int& klen) throw () {
  klen = 0;
  for (unsigned int i = 0; i < socket.num_messages(30); i++) {
    if (socket.get(30, retbuf, retlen, i) && retlen>= 7) {
      klen = retbuf[6] ? retbuf[6] : klen;
    }
  }
  if (socket.get (30,retbuf,retlen) && retlen>=7) {
    vx=0.001*read_signed_short (retbuf);
    vy=0.001*read_signed_short (retbuf+2);
    vphi=0.001*read_signed_short (retbuf+4);
    return true;
  }
  return false;
}
bool SimulatorUDPCommunication::putObstacles (const std::vector<double>& tm) throw (std::bad_alloc) {
  char buffer [2*tm.size()];
  for (unsigned int i=0; i<tm.size(); i++)
    write_signed_short (buffer+2*i, tm[i]);
  return socket.put (40,buffer,2*tm.size());
}
bool SimulatorUDPCommunication::getObstacles (std::vector<double>& tm) throw (std::bad_alloc) {
  if (socket.get (40,retbuf,retlen) && retlen>=4) {
    tm.resize (retlen/2);
    for (unsigned int i=0; i<tm.size(); i++)
      tm[i]=read_signed_short (retbuf+2*i);
    return true;
  }
  return false;
}
