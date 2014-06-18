
#ifndef _TribotsSim_dynamics_h_
#define _TribotsSim_dynamics_h_

#include <ode/ode.h>

namespace TribotsSim {

  void nearCallback (void*, dGeomID o1, dGeomID o2);

  void simLoop (int pause);

  void applyDynamics ();

  void communicate ();
  
}

#endif
