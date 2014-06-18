
#include "external.h"
#include "global.h"
#include <cstdio>
#include <cmath>
#include <iostream>
#include <vector>

using namespace TribotsSim;
using namespace std;

namespace {

  unsigned int goaldelay=0;
  unsigned int lastactiverobot=0;
  std::vector<dReal> simstates [3];
  
}


void TribotsSim::help () {
  printf ("   a/A links\n");
  printf ("   d/D rechts\n");
  printf ("   w/W vorwärts.\n");
  printf ("   s/S rückwärts.\n");
  printf ("   k links drehen.\n");
  printf ("   l rechts drehen.\n");
  printf ("   r ball reset.\n");
  printf ("   v ball aus einiger entfernung anrollen lassen.\n");
  printf ("   g ball von Mitte  "
              " mit Vektor (3,1) anrollen lassen.\n");
  printf ("   f beide Teams Formation einnehmen.\n");
  printf ("   n/N aktiven Roboter vor/zurückcycelnn\n");
  printf ("   b Move ball/robot toggle\n");
  printf ("   t create new robot\n");
  printf ("   1|2|3 make snapshot\n");
  printf ("   <caps> + 1|2|3 load snapshot\n");
  printf ("   P save snapshot to disc\n");
  printf ("   L load snapshot from disc\n");
  printf ("   Q quit\n");
}






void TribotsSim::applyRules (World& world) {
  const dReal* ballpos = world.ball.getPosition ();

  if ((ballpos[0]>-FIELD_LENGTH/2)&&(ballpos[0]<FIELD_LENGTH/2) && ballpos[1]>FIELD_WIDTH/2+0.10) {
    world.ball.setPosition (ballpos[0]/*+((rand()%10)-5)/10.0*/,FIELD_WIDTH/2,1.0);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    cout<<"THROW IN"<<endl;
  } else
  if ((ballpos[0]>-FIELD_LENGTH/2)&&(ballpos[0]<FIELD_LENGTH/2) && ballpos[1]<-FIELD_WIDTH/2-0.10) {
    world.ball.setPosition (ballpos[0]/*+((rand()%10)-5)/10.0*/,-FIELD_WIDTH/2,1.0);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    cout<<"THROW IN"<<endl;
  } else
  if ((ballpos[0]<-FIELD_LENGTH/2) && (ballpos[0]>-FIELD_LENGTH/2-50) &&  fabs(ballpos[1])<1) {
    if (goaldelay==0)
      goaldelay++;
  } else
  if ((ballpos[0]>FIELD_LENGTH/2) && (ballpos[0]<FIELD_LENGTH/2+50) &&  fabs(ballpos[1])<1) {
    if (goaldelay==0)
      goaldelay++;
  } else
  if ((ballpos[0]>FIELD_LENGTH/2) && ballpos[1]>0) {
    world.ball.setPosition (FIELD_LENGTH/2-0.5,FIELD_WIDTH/2,0.4);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    cout << "CORNER 1"<<endl;
  } else    
  if ((ballpos[0]>FIELD_LENGTH/2) && ballpos[1]<0) {
    world.ball.setPosition (FIELD_LENGTH/2-0.5,-FIELD_WIDTH/2,0.4);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    cout << "CORNER 2"<<endl;
  } else
  if ((ballpos[0]<-FIELD_LENGTH/2) && ballpos[1]>0) {
    world.ball.setPosition (-FIELD_LENGTH/2+0.5,FIELD_WIDTH/2,0.4);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    cout << "CORNER 3"<<endl;
  } else
  if ((ballpos[0]<-FIELD_LENGTH/2) && ballpos[1]<0) {
    world.ball.setPosition (-FIELD_LENGTH/2+0.5,-FIELD_WIDTH/2,0.4);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    cout << "CORNER 4"<<endl;
  }

  if (goaldelay>0)
    goaldelay++;
  if (goaldelay>30) {
    cout << "GOAL " << (ballpos[0]>0 ? "YELLOW" : "BLUE") << endl;
    world.ball.setPosition (0,0,1.0);
    world.ball.setLinearVelocity (0,0,0);
    world.ball.setAngularVelocity (0,0,0);
    goaldelay=0;
  }
}


void TribotsSim::setFormation (World& world) {
  for (unsigned int i=0; i<world.robots.size(); i++) {
    world.robots[i].setPosition (-FIELD_LENGTH/2+i, FIELD_WIDTH/2+1.5, 0.5);
    world.robots[i].setLinearVelocity (0,0,0);
    world.robots[i].setAngularVelocity (0,0,0);
  }
}


void TribotsSim::keyboardCommand (int cmd) {
  try{
    World& world (*TribotsSim::global_world_pointer);
    const dReal* ballpos = world.ball.getPosition ();
    switch (cmd) {
      case 'b':
        {
          bool ballwasactive =world.ball.isActive();
          world.ball.setActive (!ballwasactive);
          if (ballwasactive) {
            if (lastactiverobot>=world.robots.size())
              lastactiverobot=0;
            if (world.robots.size()>0)
              world.robots[lastactiverobot].setActive (true);
          }
        }
        break;
      case 'n':
        if (world.ball.isActive())
          break;
        if (lastactiverobot<world.robots.size())
          world.robots[lastactiverobot].setActive(false);
        if (world.robots.size()>0) {
          lastactiverobot=(lastactiverobot+1)%world.robots.size();
          world.robots[lastactiverobot].setActive(true);
        }
        break;
      case 'N':
        if (world.ball.isActive())
          break;
        if (lastactiverobot<world.robots.size())
          world.robots[lastactiverobot].setActive(false);
        if (world.robots.size()>0) {
          lastactiverobot=(lastactiverobot+world.robots.size()-1)%world.robots.size();
          world.robots[lastactiverobot].setActive(true);
        }
        break;
      case 'r':
        world.ball.setPosition (0,-1,1);
        world.ball.setLinearVelocity (0,0,0);
        world.ball.setAngularVelocity (0,0,0);
        break;
      case 'v':
        world.ball.setPosition (0,0,1);
        world.ball.setLinearVelocity ((rand()%20+10)*0.1,(rand()%10-5)*0.1f,0);
        world.ball.setAngularVelocity (0,0,0);
        break;
      case 'g':
        if (world.robots.size()>0) {
           //const dReal* pos = world.robots[lastactiverobot].getPosition();
           world.ball.setPosition (0,0,0.4);
           world.ball.setLinearVelocity (3,
                                         1,0);
           world.ball.setAngularVelocity (0,0,0);
        }
        break;
      case 'f':
        setFormation (world);
        break;
      case 'A':
        world.ball.addForce (-1,0,0);
        break;
      case 'D':
        world.ball.addForce (+1,0,0);
        break;
      case 'S':
        world.ball.addForce (0,-1,0);
        break;
      case 'W':
        world.ball.addForce (0,+1,0);
        break;
      case 'w':
        if (world.ball.isActive()) {
          world.ball.setPosition (ballpos[0], ballpos[1]+0.5, ballpos[2]);
          world.ball.setLinearVelocity (0,0,0);
          world.ball.setAngularVelocity (0,0,0);
        } else {
          for (unsigned int i=0; i<world.robots.size(); i++) {
            if (world.robots[i].isActive()) {
              world.robots[i].addRelativeForce (180,0,0);
            }
          }
        }
        break;
      case 's':
        if (world.ball.isActive()) {
          world.ball.setPosition (ballpos[0], ballpos[1]-0.5, ballpos[2]);
          world.ball.setLinearVelocity (0,0,0);
          world.ball.setAngularVelocity (0,0,0);
        } else {
          for (unsigned int i=0; i<world.robots.size(); i++) {
            if (world.robots[i].isActive()) {
              world.robots[i].addRelativeForce (-180,0,0);
            }
          }
        }
        break;
      case 'a':
        if (world.ball.isActive()) {
          world.ball.setPosition (ballpos[0]-0.5, ballpos[1], ballpos[2]);
          world.ball.setLinearVelocity (0,0,0);
          world.ball.setAngularVelocity (0,0,0);
        } else {
          for (unsigned int i=0; i<world.robots.size(); i++) {
            if (world.robots[i].isActive()) {
              world.robots[i].addRelativeForce (0,180,0);
            }
          }
        }
        break;
      case 'd':
        if (world.ball.isActive()) {
          world.ball.setPosition (ballpos[0]+0.5, ballpos[1], ballpos[2]);
          world.ball.setLinearVelocity (0,0,0);
          world.ball.setAngularVelocity (0,0,0);
        } else {
          for (unsigned int i=0; i<world.robots.size(); i++) {
            if (world.robots[i].isActive()) {
              world.robots[i].addRelativeForce (0,-180,0);
            }
          }
        }
        break;
      case 'j':
        for (unsigned int i=0; i<world.robots.size(); i++) {
          if (world.robots[i].isActive()) {
            world.robots[i].addRelativeTorque (0,0,6);
          }
        }
        break;        
      case 'k':
        for (unsigned int i=0; i<world.robots.size(); i++) {
          if (world.robots[i].isActive()) {
            world.robots[i].addRelativeTorque (0,0,-6);
          }
        }
        break;
      case '1':
        world.recoverSnapshot (simstates[0]);
        break;
      case '2':
        world.recoverSnapshot (simstates[1]);
        break;
      case '3':
        world.recoverSnapshot (simstates[2]);
        break;
      case '!':
        simstates[0].clear();
        world.makeSnapshot (simstates[0]);
        break;
      case '"':
        simstates[1].clear();
        world.makeSnapshot (simstates[1]);
        break;
      case '§':
        simstates[2].clear();
        world.makeSnapshot (simstates[2]);
        break;
      case 'P':
        {
          std::vector<dReal> state;
          world.makeSnapshot (state);
          saveSnapshot (state, "odesim.snap");
        }
        break;
      case 'L':
        {
          std::vector<dReal> state;
          loadSnapshot (state, "odesim.snap");
          world.recoverSnapshot (state);
        }
        break;
      case 'Q':
        quit_request=true;
#ifndef NO_X
        dsStop();
#endif
        break;
      case 't':
        {
          Tribot rob;
          rob.create (world.wid, world.sid);
          rob.setPosition (0, FIELD_WIDTH/2+1, 0.5);
          world.robots.push_back (rob);
        }
        break;
      default:
        break;
    }
  }catch(std::exception& e){
    cerr << "Fehler:\n";
    cerr << e.what() << endl;
  }
}


