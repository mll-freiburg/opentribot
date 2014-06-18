// File matrix_norms.h:
// contains the declaration of matrix and vector norms and distances
// created 13-NOV-00 by Martin Lauer
// ---------------------------------------------

#ifndef matrix_norms_h
#define matrix_norms_h

#include "matrix.h"

// in the annotations we use A, B, C, ... as abbrevations for the
// first, second, third, ... argument of a function. Most functions
// do exception handling, i.e. they throw an exception if the size
// of the matrices don't match.

namespace Martin {

  // euklid: ||X|| = sqrt(sum (x_i^2))
  double euklid_norm (const ColumnVector&) throw ();
  double euklid_distance (const ColumnVector&, const ColumnVector&) throw (std::invalid_argument);
  double euklid_norm (const RowVector&) throw ();
  double euklid_distance (const RowVector&, const RowVector&) throw (std::invalid_argument);

  // manhattan: ||X|| = sum (abs (x_i))
  double manhattan_norm (const ColumnVector&) throw ();
  double manhattan_distance (const ColumnVector&, const ColumnVector&) throw (std::invalid_argument);
  double manhattan_norm (const RowVector&) throw ();
  double manhattan_distance (const RowVector&, const RowVector&) throw (std::invalid_argument);

  // maximum: ||X|| = max (abs (x_i))
  double maximum_norm (const ColumnVector&) throw ();
  double maximum_distance (const ColumnVector&, const ColumnVector&) throw (std::invalid_argument);
  double maximum_norm (const RowVector&) throw ();
  double maximum_distance (const RowVector&, const RowVector&) throw (std::invalid_argument);

  // minkowsky (d): ||X|| = (sum (abs(x_i)^d))^(1/d)
  double minkowsky_norm (const ColumnVector&, double) throw (std::invalid_argument);
  double minkowsky_distance (const ColumnVector&, const ColumnVector&, double) throw (std::invalid_argument);
  double minkowsky_norm (const RowVector&, double) throw (std::invalid_argument);
  double minkowsky_distance (const RowVector&, const RowVector&, double) throw (std::invalid_argument);

}

#endif
