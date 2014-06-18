
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

using namespace std;
using namespace TribotsSim;

namespace TribotsSim {
  World* global_world_pointer=NULL;
  Communication* global_communication_pointer=NULL;
  bool quit_request=false;
}

int main (int argc, char** argv) {
  try{
    TribotsSim::World simulator_world;
    TribotsSim::global_world_pointer = &simulator_world;

    Communication communication;
    TribotsSim::global_communication_pointer = &communication;
    
    dWorldSetGravity (simulator_world.wid,0,0,-GRAVITY);
#ifndef NO_X  
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &TribotsSim::help;
    fn.step = &TribotsSim::simLoop;
    fn.command = &TribotsSim::keyboardCommand;

    fn.stop = 0;
    fn.path_to_textures = "drawstuff/textures";

    setFormation (simulator_world);
    dsSimulationLoop (argc,argv,352,288,&fn);

#else
    setFormation (simulator_world);
    while (!quit_request) {
      simLoop (0);
    }
#endif

  }catch(exception& e) {
    cerr << e.what() << endl;
    return -1;
  }
  return 0;
}
