
#include "dynamics.h"
#include "external.h"
#include "helpers.h"
#include "global.h"
#include "World.h"
#include "Communication.h"
#include <cmath>

using namespace TribotsSim;
using namespace std;

namespace {

  const unsigned int MAX_CONTACTS = 3;
  const double MU = 0.1;

}

void TribotsSim::nearCallback (void*, dGeomID o1, dGeomID o2) // Kollisionen zwischen allen objekten und dem boden
{
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (unsigned int i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = MU;
    //contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.7;
    contact[i].surface.bounce_vel = 0.2;
    contact[i].surface.soft_cfm = 0.05;
  }
  if (unsigned int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom, sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
//    const dReal ss[3] = {0.02,0.02,0.02};
    for (unsigned int i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (global_world_pointer->wid,global_world_pointer->cid,contact+i);
      dJointAttach (c,b1,b2);
    }
  }
}

namespace {
  Tribots::Time latestUpdateTime;
}
void TribotsSim::simLoop (int pause) {
  latestUpdateTime.update();
  TribotsSim::applyRules(*global_world_pointer);
  if (!pause)
    TribotsSim::applyDynamics ();
  global_world_pointer->draw();
  

  int cycleTime = latestUpdateTime.elapsed_usec();
  int remainTime = 33333-cycleTime;
  if (remainTime>1000)
    usleep(remainTime);
}




void TribotsSim::applyDynamics () {
  World& world (*global_world_pointer);

  dReal angularv [3];
  dReal linearv [3];
  dReal bangularv [3];
  dReal blinearv [3];
  dReal ballpos [3];
  dReal robotpos [3];
  dReal robotlook [3];
  
  double drag_linear = 40;
  double drag_angular = 1.2;
  double balldrag_linear = -2*0.005101;
  double balldrag_angular = -0.5*0.005101;

  copyVec3 (ballpos, world.ball.getPosition());
  bool ball_on_ground=(ballpos[2]<0.5);
//  cerr << "Ballh=" << ballpos[2] << " " << (ball_on_ground ? "bog" : "---");
  if (ball_on_ground) {
    // Reibung des Balls mit Boden
    copyVec3 (blinearv, world.ball.getLinearVelocity());
    copyVec3 (bangularv, world.ball.getAngularVelocity());
    dReal bvel = sqrt(blinearv[0]*blinearv[0]+blinearv[1]*blinearv[1]);
    dReal factor = 0.1*bvel*sqrt(bvel)+1;
    world.ball.addForce (factor*blinearv[0]*balldrag_linear,factor*blinearv[1]*balldrag_linear,0);
    world.ball.addTorque (balldrag_angular*bangularv[0],balldrag_angular*bangularv[1],balldrag_angular*bangularv[2]);
  }
  
  for (unsigned int i=0; i<world.robots.size(); i++) {
    // Sollgeschwindigkeit erzeugt beschleunigende Kraft
    world.robots[i].addRelativeForce (world.robots[i].getSteering()[0]*30*1.1, world.robots[i].getSteering()[1]*30*1.1, 0);
    world.robots[i].addRelativeTorque (0,0,world.robots[i].getSteering()[2]);

    // Reibung Roboter/Boden    
    copyVec3 (linearv, world.robots[i].getLinearVelocity ());
    copyVec3 (angularv, world.robots[i].getAngularVelocity ());
    world.robots[i].addForce (-linearv[0]*drag_linear, -linearv[1]*drag_linear,-linearv[2]*drag_linear);
    world.robots[i].addTorque (-angularv[0]*drag_angular,-angularv[1]*drag_angular,-angularv[2]*drag_angular);

    copyVec3 (robotpos, world.robots[i].getPosition ());
    copyVec3 (robotlook, world.robots[i].getRotation ());

    double robot_ball_distance = sqrt((robotpos[0]-ballpos[0])*(robotpos[0]-ballpos[0])+(robotpos[1]-ballpos[1])*(robotpos[1]-ballpos[1]));
    double robot_ball_angle = atan2 (ballpos[1]-robotpos[1], ballpos[0]-robotpos[0])-atan2 (-robotlook[1], robotlook[0]);
    while (robot_ball_angle<-M_PI)
      robot_ball_angle+=2*M_PI;
    while (robot_ball_angle>M_PI)
      robot_ball_angle-=2*M_PI;
    if (fabs(robot_ball_angle*180/M_PI)<20 && robot_ball_distance<0.43) {
//      cerr << "  Hoernchen ";
      // Ball zwischen den Hoernchen
      if (fabs(world.robots[i].getSteering()[3])>=0.001) {
//        cerr << " kick ";
        // Roboter schiesst
        double scaling = 5/(robot_ball_distance+1e-300);
        world.ball.setLinearVelocity (scaling*(ballpos[0]-robotpos[0]), scaling*(ballpos[1]-robotpos[1]), 0.1);
      } else {
//        cerr << " hbr ";
        // Ball wird durch die Hoernchen etwas abgebremst
        world.ball.addForce (0,0,-5);
        world.ball.addForce (blinearv[0]*balldrag_linear*30,blinearv[1]*balldrag_linear*30,0);
      }      
    }
//    cerr << endl;
  }

  dSpaceCollide (world.sid,0,&TribotsSim::nearCallback);
  dWorldQuickStep (world.wid,0.033);
  dJointGroupEmpty (world.cid);
}
