
#include "Communication.h"
#include "global.h"
#include <cmath>

using namespace TribotsSim;
using namespace std;

Communication::Communication () {
  comm.init_as_server (52781);
}

Communication::~Communication () {
  for (unsigned int i=0; i<partner_active.size(); i++) {
    if (partner_active[i]) {
      comm.clear_send_buffer ();
      comm.putBye ();
      comm.sendto (partner_addresses[i]);
    }
  }
}
  
void Communication::communicate (World& world) {
  unsigned int trials=0;
  unsigned int num_active_clients=0;
  for (unsigned int i=0; i<partner_active.size(); i++) {
    if (partner_active[i]) {
      num_active_clients++;
    }
  }
  while (true && trials<=num_active_clients) {
    trials++;
    unsigned int numpackets = comm.receive ();
    if (numpackets==0)
      break;
    const struct sockaddr_in& address = comm.partner_address();
    int senderindex=-1;
    for (unsigned int i=0; i<partner_addresses.size(); i++) {
      if (partner_addresses[i]==address) {
        senderindex=i;
        break;
      }
    }
    if (senderindex<0) {
      // bisher unbekannter Sender, erst mal einen Roboter damit verbinden
      unsigned int freerobot=world.robots.size();
      Tribots::Time now;
      for (unsigned int i=0; i<world.robots.size(); i++) {
        if (i>=partner_active.size()) {
          partner_active.push_back (false);
          partner_addresses.push_back (address);
          partner_timeout.push_back (now);
        }
        if (!partner_active[i]) {
          freerobot=i;
          break;
        }
      }
      if (freerobot>=world.robots.size()) {
        Tribot rob;
        rob.create (world.wid, world.sid);
        rob.setPosition (-FIELD_LENGTH/2+freerobot, FIELD_WIDTH/2+1, 0.5);
        world.robots.push_back (rob);
        partner_active.push_back (true);
        partner_addresses.push_back (address);
        partner_timeout.push_back (now);
      }
      cerr << "COMM: assigning robotcontrol to robot no " << freerobot << endl;
      partner_active[freerobot]=true;
      partner_addresses[freerobot]=address;
      senderindex=freerobot;
    }
    if (!partner_active[senderindex]) {
      partner_active[senderindex]=true;
      cerr << "COMM: robotcontrol reconnect for robot no " << senderindex << endl;
    }

    // Empfangene Nachrichten in die Simulation einfliessen lassen
    partner_timeout[senderindex].update();
    double x=0,y=0,z=0,phi=0,vx=0,vy=0,vz=0,vphi=0;
    bool kick=false;
    unsigned int klen=0;
    if (comm.getRobot (x,y,phi,vx,vy,vphi,kick)) {
      world.robots[senderindex].setPosition (-y/1000, x/1000, 0.5);
      world.robots[senderindex].setRotation (0, 0, phi+M_PI);
      world.robots[senderindex].setLinearVelocity (-vy, vx, 0);
      world.robots[senderindex].setAngularVelocity (0,0,vphi);
    }
    if (comm.getBall (x,y,z,vx,vy,vz)) {
      world.ball.setPosition (-y/1000, x/1000, 0.2);
      world.ball.setLinearVelocity (-vy, vx, 0);
      world.ball.setAngularVelocity (0,0,vphi);
    }
    if (comm.getDriveVector (vx,vy,vphi,klen)) {
      world.robots[senderindex].setSteeringX (vy);
      world.robots[senderindex].setSteeringY (-vx);
      world.robots[senderindex].setSteeringPhi (vphi);
      world.robots[senderindex].setSteeringKick (klen>0);
    }
    if (comm.getBye ()) {
      partner_active[senderindex]=false;
      world.robots[senderindex].setNoSteering ();
      cerr << "COMM: robot " << senderindex << " said BYE" << endl;
    }
  }

  // Kommunikationspartner auf Aktivitaet hin ueberpruefen (Timeout pruefen)
  for (unsigned int i=0; i<partner_timeout.size(); i++) {
    if (partner_active[i] && partner_timeout[i].elapsed_msec()>2000) {
      partner_active[i]=false;
      world.robots[i].setNoSteering ();
      cerr << "COMM: robot " << i << " inactive" << endl;
    }
  }

  // An alle aktiven Partner Informationen verschiecken
  for (unsigned int receiver=0; receiver<partner_active.size(); receiver++) {
    if (partner_active[receiver]) {
      comm.clear_send_buffer ();

      double x=0,y=0,z=0,phi=0,vx=0,vy=0,vz=0,vphi=0;
      bool kick=false;
      const dReal* v = world.robots[receiver].getPosition();
      x=1000*v[1];
      y=-1000*v[0];
      v = world.robots[receiver].getRotation();
      phi=atan2 (-v[1], v[0])+M_PI;
      v=world.robots[receiver].getLinearVelocity();
      vx=v[1];
      vy=-v[0];
      v=world.robots[receiver].getAngularVelocity();
      vphi=v[2];
      kick=(fabs(world.robots[receiver].getSteering()[3])>0.001);
      comm.putRobot (x,y,phi,vx,vy,vphi,kick);
      
      v=world.ball.getPosition();
      x=1000*v[1];
      y=-1000*v[0];
      z=1000*v[2];
      v=world.ball.getLinearVelocity();
      vx=v[1];
      vy=-v[0];
      vz=v[2];
      comm.putBall (x,y,z,vx,vy,vz);

      vector<double> tloc (2*(world.robots.size()-1));
      unsigned int index=0;
      for (unsigned int i=0; i<world.robots.size(); i++) {
        if (i!=receiver) {
          v=world.robots[i].getPosition();
          tloc[index++]=1000*v[1];
          tloc[index++]=-1000*v[0];
        }
      }
      comm.putObstacles (tloc);

      comm.sendto (partner_addresses[receiver]);      
    }
  }
}
