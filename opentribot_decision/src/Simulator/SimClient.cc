
#include <cmath>
#include <iostream>
#include "../WorldModel/WorldModel.h"
#include "SimulatorUDPCommunication.h"
#include "../Structures/Journal.h"
#include "SimClient.h"

using namespace std;

bool Tribots::SimClient::received_anything=false;
std::string Tribots::SimClient::host="localhost";
unsigned int Tribots::SimClient::port=52781;

#ifndef USE_SIMSRV
#define USE_SIMSRV 0
#endif

#ifndef USE_ODESIM
#define USE_ODESIM 0
#endif

#ifndef USE_ODESIM1
#define USE_ODESIM1 0
#endif

#if USE_SIMSRV
#include "simsrv2/tribotClient/simInterface.h"
#endif

#if USE_ODESIM1
#include "odeserver/odesimcomm.h"
using namespace OdeServer;
namespace {
  OdeSimComm * odesimcomm = NULL;
}
#endif

namespace {
  TribotsSim::SimulatorUDPCommunication simUDPcom;
}

Tribots::SimClient* Tribots::SimClient::the_only_sim_client (NULL);

using namespace Tribots;

void SimClient::set_host (const char* hst) throw (std::bad_alloc) {
  host=hst;
}

SimClient* SimClient::get_sim_client (const char* conf) throw (std::bad_alloc, std::invalid_argument) {
  if (!the_only_sim_client)
    the_only_sim_client = new SimClient (conf);
  return the_only_sim_client;
}

void SimClient::delete_sim_client () throw () {
  simUDPcom.close();
  if (the_only_sim_client)
    delete the_only_sim_client;
}

SimClient::SimClient (const char* conf) throw (std::bad_alloc, std::invalid_argument) : obstacle_positions (20) {
#if USE_ODESIM1
  odesimcomm=new OdeSimComm();
  odesimcomm->use_as_server(0);
  odesimcomm->use_as_client(host.c_str(),30001);
  odesimcomm->use_nonblock();
  odesimcomm->send(); // Send something so the server knows we are here and the port and inet adress
  obstacle_positions.resize (0);
#endif
#if USE_SIMSRV
  if (!SimInit(conf))
    throw std::invalid_argument ("Cannot connect to simulator");
  SimSetMotorOn();
  SimSetOmniVelocity(0,0,0);
  obstacle_positions.resize (0);
#endif
  simUDPcom.init_as_client (host.c_str(), port);
  simUDPcom.putPing ();
  simUDPcom.send();  // sich bemerkbar machen
  received_anything=false;
}

SimClient::~SimClient () throw () {
#if USE_SIMSRV
  SimSetRobotPosition (100000,100000,0);   // auf "Parkposition" setzen, da Roboter als Hindernis weiterlebt
  SimUpdate();
  SimExit();
#endif
}

void SimClient::set_drive_vector (DriveVector dv) throw () {
#if USE_ODESIM1
 odesimcomm->simstate->steer[1]=-dv.vtrans.x;
 odesimcomm->simstate->steer[0]=dv.vtrans.y;
 odesimcomm->simstate->steer[2]=dv.vrot;
 odesimcomm->simstate->kick=dv.kick;
#endif 
#if USE_SIMSRV
  SimSetOmniVelocity (static_cast<int>(1000*dv.vtrans.y), -static_cast<int>(1000*dv.vtrans.x), static_cast<int>(1000*dv.vrot));  // hier bin ich mir nicht sicher, ob Faktor 1000 ueberall angebracht ist und Koordinatensysteme uebereinstimmen
  if (dv.kick)
    SimSetKickerKick ();
  else
    SimSetKickerReleased();
#endif
  simUDPcom.putDriveVector (dv.vtrans.x, dv.vtrans.y, dv.vrot, dv.kick ? dv.klength : 0);
}

void SimClient::update () throw () {
#if USE_ODESIM1
  odesimcomm->send();
  
  int packets = 0;
  int retval;
  while((retval = odesimcomm->receive()) || packets == 0) {
    if (!retval) {   // could assert here: packets == 0
      usleep(1000);  // Wait for new information. Good idea? I'm not sure...
      continue;
    }
    packets++;
    timeval ts1;
    gettimeofday(&ts1,NULL);
    
    // Roboterposition
    robot_position.x=odesimcomm->simstate->pos[1]*1000;
    robot_position.y=-odesimcomm->simstate->pos[0]*1000;
    robot_heading.set_rad(odesimcomm->simstate->pos[2]+M_PI);
    timestamp.set (ts1);
    
    // Ballposition
    ball_position = Vec(odesimcomm->simstate->ball[1]*1000,-odesimcomm->simstate->ball[0]*1000);
    
    unsigned int n = odesimcomm->simstate->num_obstacles;
    obstacle_positions.resize (n);
    unsigned int i=0;
    unsigned int p=0;
    while (i<n) {
      // eigene Position nicht als Hindernis erkennen
      if (
          (fabs(odesimcomm->simstate->pos[0]-odesimcomm->simstate->obstacles[0][i])>0.2)
          ||
          (fabs(odesimcomm->simstate->pos[1]-odesimcomm->simstate->obstacles[1][i])>0.2)
          ) {
        Vec obstacle_position = Vec(odesimcomm->simstate->obstacles[1][i]*1000,-odesimcomm->simstate->obstacles[0][i]*1000);
        obstacle_positions[p++]=obstacle_position;
      }
      i++;
    }
    obstacle_positions.resize (p); 
  }
  if (packets == 0) {
    LOUT << "SimClient: Received nothing." << endl;
  }
  else if (packets > 1) {
    LOUT << "SimClient: Had to drop " << packets-1 << " old packets." << endl;
  }
    
#endif
#if USE_SIMSRV
  SimUpdate();
  int x1, y1, phi1;
  timeval ts1;

  // Roboterposition
  SimGetAbsOdometryPosition (x1,y1,phi1,ts1);  
  robot_position.x=static_cast<double>(-y1);
  robot_position.y=static_cast<double>(x1);
  robot_heading.set_deg (phi1);
  timestamp.set (ts1);

  // Ballposition
  SimGetBall (x1,phi1,ts1);
  Vec rel_ball = Vec(0,static_cast<double>(x1)).rotate (Angle::deg_angle (static_cast<double>(phi1)));
  ball_position = robot_position+rel_ball.rotate(robot_heading);

  // Hindernisse (wirklich wahr, nicht die Feldlinien, siehe tribotClient)
  unsigned int arr [200];
  unsigned int n = SimGetFieldlines(200,arr,ts1);
  unsigned int p = 0;
  obstacle_positions.resize (n/4);
  // arr enthaelt die Roboterpositionen, je Roboter: teamcolor, Sichtwinkel, Entfernung, don't-care
  unsigned int i=0;
  while (i<n) {
    i++;  // teamcolor ignorieren
    int win = arr[i++];
    unsigned int dist = arr[i++];
    Angle phi1 =Angle::deg_angle (win);
    double distance = static_cast<double>(dist);
    i++;  // don't care
    if (distance>15) {
      // eigene Position nicht als Hindernis erkennen
      Vec rel_obstacle = Vec(0,distance).rotate (phi1);
      Vec obstacle_position = robot_position+rel_obstacle.rotate(robot_heading);
      obstacle_positions[p++]=obstacle_position;
    }
  }
  obstacle_positions.resize (p);
#endif

  if (received_anything)
    simUDPcom.send();
  else
    simUDPcom.clear_send_buffer();

  bool rp, bp, op;
  bool rec = (simUDPcom.receive ()>0);
  if (rec) {
    if (!received_anything)
      JWARNING ("connection to simulator established");
    received_anything = true;
    timestamp.update();
  }
  double d1, d2, d3, d4;
  bool b1;
  std::vector<double> dd;
  rp=simUDPcom.getRobot (robot_position.x, robot_position.y, d1, robot_linear_velocity.x, robot_linear_velocity.y, robot_angular_velocity, b1);
  if (rp)
    robot_heading.set_rad (d1);
  bp=simUDPcom.getBall (ball_position.x, ball_position.y, d1, d2, d3, d4);
  op=simUDPcom.getObstacles (dd);
  if (op) {
    obstacle_positions.resize (dd.size()/2);
    for (unsigned int i=0; i<obstacle_positions.size(); i++) {
      obstacle_positions[i].x=dd[2*i];
      obstacle_positions[i].y=dd[2*i+1];
    }
    latest_obstacle_timestamp.update();
  } else if (latest_obstacle_timestamp.elapsed_msec()>500) {
    obstacle_positions.resize(0);
  }
  if (rec)
    timestamp.update ();
  if (simUDPcom.getBye () || timestamp.elapsed_msec()>3000) {
    if (simUDPcom.getBye ())
      JWARNING ("connection to simulator closed by simulator");
    else if (received_anything)
      JWARNING ("connection to simulator lost");
    // Socket aufgeben
    received_anything=false;
    simUDPcom.free_socket();
    
    // neue Verbindung aufzubauen versuchen
    simUDPcom.init_as_client (host.c_str(), port);
    simUDPcom.putPing ();
    simUDPcom.send();  // sich bemerkbar machen
    timestamp.update();
  }
}
