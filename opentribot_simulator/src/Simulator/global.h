
#ifndef _TribotsSim_global_h_
#define _TribotsSim_global_h_

// Hack, globale Definitionen

const double BALL_RADIUS = 0.11;
const double FIELD_LENGTH = 18;
const double FIELD_WIDTH = 12;
const double CENTER_CIRCLE_RADIUS=2;
const double GOAL_AREA_LENGTH=0.75;
const double GOAL_AREA_WIDTH=3.5;
const double PENALTY_AREA_LENGTH=2.25;
const double PENALTY_AREA_WIDTH=6.5;
const double CORNER_ARC_RADIUS=0.75;
const double PENALTY_AREA_SPOT=3;
const double GRAVITY = 0.9;

#ifdef dDOUBLE
#include <drawstuff/drawstuff.h>
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

namespace TribotsSim {
  class World;
  class Communication;
  extern World* global_world_pointer;
  extern Communication* global_communication_pointer;
  extern bool quit_request;
}


#endif
