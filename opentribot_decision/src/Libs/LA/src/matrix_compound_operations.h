// File matrix_compound_operations.h:
// contains the declaration of matrix compund operations
// i.e. combinations of basic matrix operations
// created 13-NOV-00 by Martin Lauer
// ---------------------------------------------

#ifndef matrix_compound_operations_h
#define matrix_compound_operations_h

#include "matrix.h"

// in the annotations we use A, B, C, ... as abbrevations for the
// first, second, third, ... argument of a function. Most functions
// do exception handling, i.e. they throw an exception if the size
// of the matrices don't match.

namespace Martin {

  // spezielle Matrizen
  void identity (DiMatrix&, double =1) throw ();  // A=b*identity

  // combinations of basic arithmetic functions:
  void multiply_with_transpose (SyMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B*B^T
  void multiply_with_transpose (SyMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B*B^T
  void multiply_with_transpose (SyMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*B^T
  void multiply_with_transpose (SyMatrix&, const LTMatrix&) throw (std::invalid_argument); // A=B*B^T
  void multiply_with_transpose (SyMatrix&, const UTMatrix&) throw (std::invalid_argument); // A=B*B^T

  void multiply_transpose_with (SyMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B^T*B
  void multiply_transpose_with (SyMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B*B^T
  void multiply_transpose_with (SyMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^T*B
  void multiply_transpose_with (SyMatrix&, const LTMatrix&) throw (std::invalid_argument); // A=B^T*B
  void multiply_transpose_with (SyMatrix&, const UTMatrix&) throw (std::invalid_argument); // A=B^T*B

  void multiply_with_transpose_with_matrix (SyMatrix&, const Matrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=B^T*C*B
  void multiply_with_matrix_with_transpose (SyMatrix&, const Matrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=B*C*B^T

  void add_with_scalars (ReMatrix&, double, const Matrix&, double, const Matrix&) throw (std::invalid_argument); // A=b*C+d*E
  void add_with_scalars (LTMatrix&, double, const LowerTriangularMatrix&, double, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=b*C+d*E
  void add_with_scalars (UTMatrix&, double, const UpperTriangularMatrix&, double, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=b*C+d*E
  void add_with_scalars (DiMatrix&, double, const DiagonalMatrix&, double, const DiagonalMatrix&) throw (std::invalid_argument); // A=b*C+d*E
  void add_with_scalars (SyMatrix&, double, const SymmetricMatrix&, double, const SymmetricMatrix&) throw (std::invalid_argument); // A=b*C+d*E

  void add_with_scalars (ReMatrix&, double, const Matrix&, double, const Matrix&, double, const Matrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G
  void add_with_scalars (LTMatrix&, double, const LowerTriangularMatrix&, double, const LowerTriangularMatrix&, double, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G
  void add_with_scalars (UTMatrix&, double, const UpperTriangularMatrix&, double, const UpperTriangularMatrix&, double, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G
  void add_with_scalars (DiMatrix&, double, const DiagonalMatrix&, double, const DiagonalMatrix&, double, const DiagonalMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G
  void add_with_scalars (SyMatrix&, double, const SymmetricMatrix&, double, const SymmetricMatrix&, double, const SymmetricMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G

  void add_with_scalars (ReMatrix&, double, const Matrix&, double, const Matrix&, double, const Matrix&, double, const Matrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G+h*I
  void add_with_scalars (SyMatrix&, double, const SymmetricMatrix&, double, const SymmetricMatrix&, double, const SymmetricMatrix&, double, const SymmetricMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G+h*I
  void add_with_scalars (LTMatrix&, double, const LowerTriangularMatrix&, double, const LowerTriangularMatrix&, double, const LowerTriangularMatrix&, double, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G+h*I
  void add_with_scalars (UTMatrix&, double, const UpperTriangularMatrix&, double, const UpperTriangularMatrix&, double, const UpperTriangularMatrix&, double, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G+h*I
  void add_with_scalars (DiMatrix&, double, const DiagonalMatrix&, double, const DiagonalMatrix&, double, const DiagonalMatrix&, double, const DiagonalMatrix&) throw (std::invalid_argument); // A=b*C+d*E+f*G+h*I


  // combinations of basic arithmetic functions, result is written in an operation's argument:
  void add_to (ReMatrix&, const Matrix&) throw (std::invalid_argument); // A=A+B
  void add_to (LTMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=A+B
  void add_to (UTMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=A+B
  void add_to (SyMatrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=A+B
  void add_to (DiMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=A+B

  void subtract_from (ReMatrix&, const Matrix&) throw (std::invalid_argument); // A=A-B
  void subtract_from (LTMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=A-B
  void subtract_from (UTMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=A-B
  void subtract_from (SyMatrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=A-B
  void subtract_from (DiMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=A-B

  void add_to_with_scalars (ReMatrix&, double, double, const Matrix&) throw (std::invalid_argument); // A=b*A+c*D
  void add_to_with_scalars (SyMatrix&, double, double, const SymmetricMatrix&) throw (std::invalid_argument); // A=b*A+c*D
  void add_to_with_scalars (LTMatrix&, double, double, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=b*A+c*D
  void add_to_with_scalars (UTMatrix&, double, double, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=b*A+c*D
  void add_to_with_scalars (DiMatrix&, double, double, const DiagonalMatrix&) throw (std::invalid_argument); // A=b*A+c*D

  void multiply_with_scalars (ReMatrix&, double) throw (); // A*=b
  void multiply_with_scalars (SyMatrix&, double) throw (); // A*=b
  void multiply_with_scalars (LTMatrix&, double) throw (); // A*=b
  void multiply_with_scalars (UTMatrix&, double) throw (); // A*=b
  void multiply_with_scalars (DiMatrix&, double) throw (); // A*=b

  void add_to_multiply_with_transpose (SyMatrix&, const Matrix&) throw (std::invalid_argument); // A=A+B*B^T
  void add_to_with_scalars_multiply_with_transpose (SyMatrix&, double, double, const Matrix&) throw (std::invalid_argument); // A=b*A+c*D*D^T

}

#endif
