
#ifndef _TribotsSim_helpers_h_
#define _TribotsSim_helpers_h_

#include <ode/ode.h>
#include <vector>

namespace TribotsSim {

  void copyVec (dReal* dest, const dReal* src, unsigned int dim); ///< kopieren zweier Arrays
  inline void copyVec2 (dReal* dest, const dReal* src) { copyVec (dest, src, 2); }
  inline void copyVec3 (dReal* dest, const dReal* src) { copyVec (dest, src, 3); }
  inline void copyVec4 (dReal* dest, const dReal* src) { copyVec (dest, src, 4); }

  void appendVecToVector (std::vector<dReal>& dest, const dReal* src, unsigned int dim);  ///< Array an vector anhaengen
  inline void appendVecToVector2 (std::vector<dReal>& dest, const dReal* src) { appendVecToVector (dest, src, 2); }
  inline void appendVecToVector3 (std::vector<dReal>& dest, const dReal* src) { appendVecToVector (dest, src, 3); }
  inline void appendVecToVector4 (std::vector<dReal>& dest, const dReal* src) { appendVecToVector (dest, src, 4); }
  
}

#endif
