// File matrix_operations.h:
// contains the declaration of basic matrix operations for linear algebra
// created 13-NOV-00 by Martin Lauer
// ---------------------------------------------

#ifndef matrix_operations_h
#define matrix_operations_h

#include "matrix.h"

// in the annotations we use A, B, C, ... as abbrevations for the
// first, second, third, ... argument of a function. Most functions
// do exception handling, i.e. they throw an exception if the size
// of the matrices don't match.

namespace Martin {
  // basic functions (copy/assign):
  void copy (ReMatrix&, const Matrix&) throw (std::invalid_argument); // A=B
  void copy (SyMatrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=B
  void copy (LTMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B
  void copy (UTMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B
  void copy (DiMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B


  // basic arithmetic functions (+, -, *, transpose (^T)):
  void add (ReMatrix&, const Matrix&, const Matrix&) throw (std::invalid_argument); // A=B+C
  void add (SyMatrix&, const SymmetricMatrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=B+C
  void add (LTMatrix&, const LowerTriangularMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B+C
  void add (UTMatrix&, const UpperTriangularMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B+C
  void add (DiMatrix&, const DiagonalMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B+C

  void subtract (ReMatrix&, const Matrix&, const Matrix&) throw (std::invalid_argument); // A=B-C
  void subtract (SyMatrix&, const SymmetricMatrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=B-C
  void subtract (LTMatrix&, const LowerTriangularMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B-C
  void subtract (UTMatrix&, const UpperTriangularMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B-C
  void subtract (DiMatrix&, const DiagonalMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B-C

  void multiply (DiMatrix&, const DiagonalMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (LTMatrix&, const LowerTriangularMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (UTMatrix&, const UpperTriangularMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (LTMatrix&, const DiagonalMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (UTMatrix&, const DiagonalMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const DiagonalMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*C
  void multiply (LTMatrix&, const LowerTriangularMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const UpperTriangularMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const LowerTriangularMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const LowerTriangularMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*C
  void multiply (UTMatrix&, const UpperTriangularMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const UpperTriangularMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const Matrix&) throw (std::invalid_argument); // A=B*C

  void multiply (LTMatrix&, const DiMatrix&, const LTMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (UTMatrix&, const DiMatrix&, const UTMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const DiMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*C
  void multiply (LTMatrix&, const LTMatrix&, const DiMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (UTMatrix&, const UTMatrix&, const DiMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const DiMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const LTMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const UTMatrix&, const Matrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const LTMatrix&, const UTMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const UTMatrix&, const LTMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const LTMatrix&) throw (std::invalid_argument); // A=B*C
  void multiply (ReMatrix&, const Matrix&, const UTMatrix&) throw (std::invalid_argument); // A=B*C

  void multiply_with_scalars (ReMatrix&, double, const Matrix&) throw (std::invalid_argument); // A=b*C
  void multiply_with_scalars (SyMatrix&, double, const SymmetricMatrix&) throw (std::invalid_argument); // A=b*C
  void multiply_with_scalars (LTMatrix&, double, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=b*C
  void multiply_with_scalars (UTMatrix&, double, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=b*C
  void multiply_with_scalars (DiMatrix&, double, const DiagonalMatrix&) throw (std::invalid_argument); // A=b*C

  void transpose (ReMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^T
  void transpose (SyMatrix&, const SymmetricMatrix&) throw (std::invalid_argument); // A=B^T
  void transpose (UTMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B^T
  void transpose (LTMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B^T
  void transpose (DiMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B^T

  // inverse matrix (^-1):
  void inv (UTMatrix&, const UpperTriangularMatrix&) throw (std::invalid_argument); // A=B^-1
  void inv (LTMatrix&, const LowerTriangularMatrix&) throw (std::invalid_argument); // A=B^-1
  void inv (SyMatrix&, const SymmetricMatrix&) throw (std::invalid_argument, std::bad_alloc); // A=B^-1, 
  // this function only works for positive definite matrices B ! Otherwise it throws an exception
  // use the function below instead, if B is not positive definite
  void inv (ReMatrix&, const Matrix&) throw (std::invalid_argument, std::bad_alloc); // A=B^-1, gauss algorithm
  void inv (DiMatrix&, const DiagonalMatrix&) throw (std::invalid_argument); // A=B^-1

  // Gauss elimination:
  void gauss (ReMatrix&, const Matrix&, const Matrix&) throw (std::invalid_argument, std::bad_alloc); // A=B^-1*C
  void gauss (ReMatrix&, const LowerTriangularMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^-1*C
  void gauss (ReMatrix&, const UpperTriangularMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^-1*C
  void gauss (ReMatrix&, const DiagonalMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^-1*C
  void gauss (ReMatrix&, const LTMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^-1*C
  void gauss (ReMatrix&, const UTMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^-1*C
  void gauss (ReMatrix&, const DiMatrix&, const Matrix&) throw (std::invalid_argument); // A=B^-1*C

  // determinants:
  double determinant (const Matrix&) throw (std::invalid_argument, std::bad_alloc); // return determinant of A
  double determinant (const UpperTriangularMatrix&) throw (); // det(A)
  double determinant (const LowerTriangularMatrix&) throw (); // det(A)
  bool is_positive_definite (const SymmetricMatrix&) throw (std::bad_alloc); // is A positive definite?

  double determinant (const LTMatrix&) throw (); // det(A)
  double determinant (const UTMatrix&) throw (); // det(A)
  double determinant (const DiMatrix&) throw (); // det(A)

  // trace:
  double trace (const Matrix&) throw (std::invalid_argument);  // trace(A)

  // dot products:
  double dot_product (const ColumnVector&, const ColumnVector&) throw (std::invalid_argument);
  double dot_product (const RowVector&, const RowVector&) throw (std::invalid_argument);
  

  // cholesky factorization:
  // if B is positive definite compute B=(L^T)*L (L is an upper triangular matrix)
  void cholesky_factorization (LTMatrix&, const SymmetricMatrix&) throw (std::invalid_argument);
  // A=L^T, if B is not positive definite the function throws an exception
  void cholesky_factorization (UTMatrix&, const SymmetricMatrix&) throw (std::invalid_argument);
  // A=L, if B is not positive definite the function throws an exception

}

#endif
