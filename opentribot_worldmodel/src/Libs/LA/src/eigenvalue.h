// File eigenvalue.h:
// contains functions to compute eigenvalues (and eigenvectors)
// created 17-DEC-01 by Martin Lauer
// ---------------------------------------------

#ifndef eigenvalue_h
#define eigenvalue_h

#include "matrix.h"

namespace Martin {

  void eigenvalue (ColumnVector&, const SymmetricMatrix&) throw (std::bad_alloc, std::invalid_argument);
  // compute the eigenvalues of a symmetric matrix
  // arg1 = result vector (dest)
  // arg2 = matrix (src)

  void eigenvalue (ColumnVector&, ReMatrix&, const SymmetricMatrix&) throw (std::bad_alloc, std::invalid_argument);
  // compute the eigenvalues and eigenvectors of a symmetric matrix
  // arg1 = result vector (dest)
  // arg2 = matrix of eigenvectors (square), the i-th column represents
  //        the eigenvector according to the i-th eigenvalue
  // arg3 = matrix (src)

}


#endif
