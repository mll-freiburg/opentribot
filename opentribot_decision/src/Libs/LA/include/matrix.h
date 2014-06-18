// File matrix.h:
// contains the declaration of matrix and vector for linear algebra
// created 13-NOV-00 by Martin Lauer
// ---------------------------------------------

#ifndef matrix_h
#define matrix_h

#include <iostream>
#include <stdexcept>

// Matrix indeces run from 1..p, 1..q !!!
// Matrix.entry (1,1) is the upper left corner
// Matrix.entry (p,1) is the lower left corner
// Matrix.entry (1,q) is the upper right corner
// Matrix.entry (p,q) is the lower right corner

// there are two classes for every type of matrix:
// -> an abstract class with read-only methods
// -> a completely implemented class with read- and write-methods
//
// list of classes:
// read-only               read and write
// Matrix                  ReMatrix                   (general size, no restrictions)
// SymmetricMatrix         SyMatrix
// LowerTriangularMatrix   LTMatrix
// UpperTriangularMatrix   UTMatrix
// DiagonalMatrix          DiMatrix
//                         ColumnVector
//                         RowVector

// for matrix operations see header <matrix_operations.h>

namespace Martin {

  class Matrix {
  // abstract class for readable matrices
  public:
    virtual ~Matrix () throw () { ; }
    virtual double entry (unsigned int, unsigned int) const throw () =0;
    virtual double at (unsigned int, unsigned int) const throw (std::out_of_range) =0;
    virtual unsigned int number_columns () const throw () =0;
    virtual unsigned int number_rows () const throw () =0;
  private:
    const Matrix& operator= (const Matrix&); 
  };
    
  class UpperTriangularMatrix {
  // abstract class for readable upper triangular matrices
  public:
    virtual ~UpperTriangularMatrix () throw () { ; }
    virtual double entry (unsigned int, unsigned int) const throw () =0;
    virtual double at (unsigned int, unsigned int) const throw (std::out_of_range) =0;
    virtual unsigned int number_columns () const throw () =0;
    virtual unsigned int number_rows () const throw () =0;
  private:
    const UpperTriangularMatrix& operator= (const UpperTriangularMatrix&); 
  };
  
  class LowerTriangularMatrix {
  // abstract class for readable lower triangular matrices
  public:
    virtual ~LowerTriangularMatrix () throw () { ; }
    virtual double entry (unsigned int, unsigned int) const throw () =0;
    virtual double at (unsigned int, unsigned int) const throw (std::out_of_range) =0;
    virtual unsigned int number_columns () const throw () =0;
    virtual unsigned int number_rows () const throw () =0;
  private:
    const LowerTriangularMatrix& operator= (const LowerTriangularMatrix&); 
  };
  
  class SymmetricMatrix {
  // abstract class for readable symmetric matrices
  public:
    virtual ~SymmetricMatrix () throw () { ; }
    virtual double entry (unsigned int, unsigned int) const throw () =0;
    virtual double at (unsigned int, unsigned int) const throw (std::out_of_range) =0;
    virtual unsigned int number_columns () const throw () =0;
    virtual unsigned int number_rows () const throw () =0;
  private:
    const SymmetricMatrix& operator= (const SymmetricMatrix&); 
  };
  
  class DiagonalMatrix {
  // abstract class for readable diagonal matrices
  public:
    virtual ~DiagonalMatrix () throw () { ; }
    virtual double entry (unsigned int, unsigned int) const throw () =0;
    virtual double at (unsigned int, unsigned int) const throw (std::out_of_range) =0;
    virtual unsigned int number_columns () const throw () =0;
    virtual unsigned int number_rows () const throw () =0;
  private:
    const DiagonalMatrix& operator= (const DiagonalMatrix&); 
  };
  
  
  
  
  
  class ReMatrix : public Matrix {
  // readable and writable general size matrix
  public:
    ReMatrix (unsigned int, unsigned int) throw (std::bad_alloc, std::invalid_argument);
    ReMatrix (const ReMatrix&) throw (std::bad_alloc);
    ReMatrix (const Matrix&) throw (std::bad_alloc);
    ~ReMatrix () throw ();
    virtual void resize (unsigned int, unsigned int) throw (std::bad_alloc);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    double entry (unsigned int, unsigned int) const throw ();
    virtual double& r_entry (unsigned int, unsigned int) throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    virtual void clear () throw ();
    virtual const ReMatrix& operator= (const Matrix&) throw (std::invalid_argument);
    virtual const ReMatrix& operator= (const ReMatrix&) throw (std::invalid_argument);
  protected:
    ReMatrix () throw ();
    /*  private: */
    double* f;
    unsigned int nr;
    unsigned int nc;
  };
  
  class ColumnVector : public ReMatrix {
  // readable and writable column vector
  public:
    ColumnVector (unsigned int) throw (std::bad_alloc, std::invalid_argument);
    ColumnVector (const ColumnVector&) throw (std::bad_alloc);
    ColumnVector (const Matrix&) throw (std::bad_alloc, std::invalid_argument);
    ColumnVector (const double*, unsigned int) throw (std::bad_alloc, std::invalid_argument);
    virtual void resize (unsigned int) throw (std::bad_alloc);
    virtual double entry (unsigned int)  const throw ();
    virtual double& r_entry (unsigned int) throw ();
    virtual double at (unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int) throw (std::out_of_range);
    virtual const ColumnVector& operator= (const ColumnVector&) throw (std::invalid_argument);
    virtual void to_c_array (double*) const throw ();
    virtual void from_c_array (const double*) throw ();
  protected:
    ColumnVector () throw ();
  };
  
  class RowVector : public ReMatrix {
  // readable and writable row vector
  public:
    RowVector (unsigned int) throw (std::bad_alloc, std::invalid_argument);
    RowVector (const RowVector&) throw (std::bad_alloc);
    RowVector (const Matrix&) throw (std::bad_alloc, std::invalid_argument);
    RowVector (const double*, unsigned int) throw (std::bad_alloc, std::invalid_argument);
    virtual void resize (unsigned int) throw (std::bad_alloc);
    virtual double entry (unsigned int) const throw ();
    virtual double& r_entry (unsigned int) throw ();
    virtual double at (unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int) throw (std::out_of_range);
    virtual const RowVector& operator= (const RowVector&) throw (std::invalid_argument);
    virtual void to_c_array (double*) const throw ();
    virtual void from_c_array (const double*) throw ();
  protected:
    RowVector () throw ();
  };
  
  class SyMatrix : public SymmetricMatrix, public Matrix {
  // readable and writable symmetric matrix
  public:
    SyMatrix (unsigned int) throw (std::bad_alloc, std::invalid_argument);
    SyMatrix (const SyMatrix&) throw (std::bad_alloc);
    SyMatrix (const SymmetricMatrix&) throw (std::bad_alloc);
    ~SyMatrix () throw ();
    virtual void resize (unsigned int) throw (std::bad_alloc);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    double entry (unsigned int, unsigned int) const throw ();
    virtual double& r_entry (unsigned int, unsigned int) throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    virtual void clear () throw ();
    virtual const SyMatrix& operator= (const SymmetricMatrix&) throw (std::invalid_argument); 
    virtual const SyMatrix& operator= (const SyMatrix&) throw (std::invalid_argument); 
  protected:
    SyMatrix () throw ();
  private:
    double* f;
    unsigned int n;
  };
  
  class LTMatrix : public LowerTriangularMatrix, public Matrix {
  // readable and writable lower triangular matrix
  public:
    LTMatrix (unsigned int) throw (std::bad_alloc, std::invalid_argument);
    LTMatrix (const LTMatrix&) throw (std::bad_alloc);
    LTMatrix (const LowerTriangularMatrix&) throw (std::bad_alloc);
    ~LTMatrix () throw ();
    virtual void resize (unsigned int) throw (std::bad_alloc);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    double entry (unsigned int, unsigned int) const throw ();
    virtual double& r_entry (unsigned int, unsigned int) throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    virtual void clear () throw ();
    virtual const LTMatrix& operator= (const LowerTriangularMatrix&) throw (std::invalid_argument);
    virtual const LTMatrix& operator= (const LTMatrix&) throw (std::invalid_argument);
  protected:
    LTMatrix () throw ();
  private:
    double* f;
    unsigned int n;
  };
  
  class UTMatrix : public UpperTriangularMatrix, public Matrix {
  // readable and writable upper triangular matrix
  public:
    UTMatrix (unsigned int) throw (std::bad_alloc, std::invalid_argument);
    UTMatrix (const UTMatrix&) throw (std::bad_alloc);
    UTMatrix (const UpperTriangularMatrix&) throw (std::bad_alloc);
    ~UTMatrix () throw ();
    virtual void resize (unsigned int) throw (std::bad_alloc);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    double entry (unsigned int, unsigned int) const throw ();
    virtual double& r_entry (unsigned int, unsigned int) throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    virtual void clear () throw ();
    virtual const UTMatrix& operator= (const UpperTriangularMatrix&) throw (std::invalid_argument);
    virtual const UTMatrix& operator= (const UTMatrix&) throw (std::invalid_argument);
  protected:
    UTMatrix () throw ();
  private:
    double* f;
    unsigned int n;
  };
  
  class DiMatrix : public DiagonalMatrix, public SymmetricMatrix, public LowerTriangularMatrix, public UpperTriangularMatrix, public Matrix {
  // readable and writable diagonal matrix
  public:
    DiMatrix (unsigned int) throw (std::bad_alloc, std::invalid_argument);
    DiMatrix (const DiMatrix&) throw (std::bad_alloc);
    DiMatrix (const DiagonalMatrix&) throw (std::bad_alloc);
    ~DiMatrix () throw ();
    virtual void resize (unsigned int) throw (std::bad_alloc);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    double entry (unsigned int, unsigned int) const throw ();
    virtual double& r_entry (unsigned int, unsigned int) throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    virtual double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    virtual void clear () throw ();
    virtual const DiMatrix& operator= (const DiagonalMatrix&) throw (std::invalid_argument);
    virtual const DiMatrix& operator= (const DiMatrix&) throw (std::invalid_argument);
  protected:
    DiMatrix () throw ();
  private:
    double* f;
    unsigned int n;
  };
  
}

std::ostream& operator<< (std::ostream&, const Martin::Matrix&);
// output of matrices in matlab format


#endif








