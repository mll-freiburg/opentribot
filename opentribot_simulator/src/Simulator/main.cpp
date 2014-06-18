
#include "World.h"
#ifndef NO_X
#include <drawstuff/drawstuff.h>
#endif
#include <ode/ode.h>
#include "global.h"
#include "external.h"
#include "dynamics.h"
#include <stdexcept>
#include <iostream>
#include "Communication.h"
#include "opentribot_messages/TribotDrive.h"
#include "opentribot_messages/WorldModel.h"

#include <ros/ros.h>



using namespace std;
using namespace TribotsSim;

dsFunctions fn;

namespace TribotsSim {
  World* global_world_pointer=NULL;
  Communication* global_communication_pointer=NULL;
  ros::Publisher* global_worldmodel_publisher; 
 bool quit_request=false;
}



void steponce(int a){

ros::spinOnce();
TribotsSim::simLoop(a);
global_communication_pointer->send_worldmodel_ros();


}



int main (int argc, char** argv) {
 ros::init(argc,argv,"simulator");
 ros::NodeHandle n;
 ros::Subscriber sub=n.subscribe("drivecommands",1,&TribotsSim::Communication::set_drive_command,global_communication_pointer);
 cout <<"Starting stuff"<<endl;
//  try{
    TribotsSim::World simulator_world;
    TribotsSim::global_world_pointer = &simulator_world;

    Communication communication;
    TribotsSim::global_communication_pointer = &communication;
    
    dWorldSetGravity (simulator_world.wid,0,0,-GRAVITY);

 
    // setup pointers to drawstuff callback functions
   // dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &TribotsSim::help;
    fn.step = &steponce;
    fn.command = &TribotsSim::keyboardCommand;

    fn.stop = 0;
    fn.path_to_textures = "drawstuff/textures";

    setFormation (simulator_world);
    dsSimulationLoop (argc,argv,352,288,&fn);
   /* while (ros::ok()) 
    {   
  	steponce(0);
    }
    */	

    setFormation (simulator_world);

 cout <<"Exiting next line."<<endl;
  return 0;
}
