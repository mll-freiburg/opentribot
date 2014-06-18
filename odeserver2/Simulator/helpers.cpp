
#include "helpers.h"

using namespace TribotsSim;

void TribotsSim::copyVec (dReal* dest, const dReal* src, unsigned int dim) {
  for (unsigned int i=0; i<dim; i++)
    dest[i]=src[i];
}

void TribotsSim::appendVecToVector (std::vector<dReal>& dest, const dReal* src, unsigned int dim) {
  for (unsigned int i=0; i<dim; i++)
    dest.push_back (src[i]);
}
