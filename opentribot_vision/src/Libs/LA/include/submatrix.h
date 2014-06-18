// File submatrix.h:
// contains the declaration of submatrices and subvectors
// created 02-MAY-01 by Martin Lauer
// ---------------------------------------------

#ifndef submatrix_h
#define submatrix_h

#include "indicator_array.h"
#include "matrix.h"

namespace Martin {

  // classes Sub__Matrix:
  // allow to address a part of a Matrix as a Matrix itself. The indicator 
  // arrays indicate which parts to take from the original matrices.
  //
  // Warning: 
  // These submatrices don't copy the values of the original matrices. 
  // Thus every change in the submatrix changes the original matrix!
  // Don't delete the original matrix before you delete the submatrix!
  // When using one of the submatrix-types as return argument of a function
  // make sure that the original matrix exists in the calling procedure!
  // The copy-constructor only copies the submatrix, not the original matrix!


  class SubMatrix : public Matrix {
  public:
    SubMatrix (const Matrix&, const IndicatorArray&, const IndicatorArray&, bool =true, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix for only reading access from the matrix (arg1) according
    // to the indicator arrays (arg2) and (arg3): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg4 and arg3.entry(j)=arg5
    SubMatrix (const SubMatrix&) throw (std::bad_alloc);
    ~SubMatrix () throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
  private:
    const Matrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int* j_ind;
    unsigned int i_size;
    unsigned int j_size;
    const SubMatrix operator= (const SubMatrix&);
  };

  class SubSymmetricMatrix : public SymmetricMatrix, public Matrix {
  public:
    SubSymmetricMatrix (const SymmetricMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix for only reading access from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubSymmetricMatrix (const SubSymmetricMatrix&) throw (std::bad_alloc);
    ~SubSymmetricMatrix () throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
  private:
    const SymmetricMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
    const SubSymmetricMatrix operator= (const SubSymmetricMatrix&);
  };

  class SubUpperTriangularMatrix : public UpperTriangularMatrix, public Matrix {
  public:
    SubUpperTriangularMatrix (const UpperTriangularMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix for only reading access from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubUpperTriangularMatrix (const SubUpperTriangularMatrix&) throw (std::bad_alloc);
    ~SubUpperTriangularMatrix () throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
  private:
    const UpperTriangularMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
    const SubUpperTriangularMatrix operator= (const SubUpperTriangularMatrix&);
  };

  class SubLowerTriangularMatrix : public LowerTriangularMatrix, public Matrix {
  public:
    SubLowerTriangularMatrix (const LowerTriangularMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix for only reading access from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubLowerTriangularMatrix (const SubLowerTriangularMatrix&) throw (std::bad_alloc);
    ~SubLowerTriangularMatrix () throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
  private:
    const LowerTriangularMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
    const SubLowerTriangularMatrix operator= (const SubLowerTriangularMatrix&);
  };

  class SubDiagonalMatrix : public DiagonalMatrix, public Matrix {
  public:
    SubDiagonalMatrix (const DiagonalMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix for only reading access from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubDiagonalMatrix (const SubDiagonalMatrix&) throw (std::bad_alloc);
    ~SubDiagonalMatrix () throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
  private:
    const DiagonalMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
    const SubDiagonalMatrix operator= (const SubDiagonalMatrix&);
  };



  class SubReMatrix : public ReMatrix {
  public:
    SubReMatrix (ReMatrix&, const IndicatorArray&, const IndicatorArray&, bool =true, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator arrays (arg2) and (arg3): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg4 and arg3.entry(j)=arg5
    SubReMatrix (const SubReMatrix&) throw (std::bad_alloc);
    ~SubReMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubReMatrix& operator= (const SubReMatrix&) throw (std::invalid_argument);
  private:
    ReMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int* j_ind;
    unsigned int i_size;
    unsigned int j_size;
  };

  class SubReSyMatrix : public ReMatrix {
  public:
    SubReSyMatrix (SyMatrix&, const IndicatorArray&, const IndicatorArray&, bool =true, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg4 and arg3.entry(j)=arg5
    // indicator arrays of rows and columns must be disjoint or overlap in at 
    // most one position
    SubReSyMatrix (SyMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)!=arg3 and arg2.entry(j)=arg3
    // ("the lower left/upper right rectangle" of a symmetric matrix)
    SubReSyMatrix (const SubReSyMatrix&) throw (std::bad_alloc);
    ~SubReSyMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubReSyMatrix& operator= (const SubReSyMatrix&) throw (std::invalid_argument);
  private:
    SyMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int* j_ind;
    unsigned int i_size;
    unsigned int j_size;
  };

  class SubSyMatrix : public SyMatrix {
  public:
    SubSyMatrix (SyMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubSyMatrix (const SubSyMatrix&) throw (std::bad_alloc);
    ~SubSyMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubSyMatrix& operator= (const SubSyMatrix&) throw (std::invalid_argument);
  private:
    SyMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
  };

  class SubReLTMatrix : public ReMatrix {
  public:
    SubReLTMatrix (LTMatrix&, const IndicatorArray&, const IndicatorArray&, bool =true, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg4 and arg3.entry(j)=arg5
    // (the lower left rectangle of a lower triangular matrix)    
    SubReLTMatrix (const SubReLTMatrix&) throw (std::bad_alloc);
    ~SubReLTMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubReLTMatrix& operator= (const SubReLTMatrix&) throw (std::invalid_argument);
  private:
    LTMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int* j_ind;
    unsigned int i_size;
    unsigned int j_size;
  };

  class SubLTMatrix : public LTMatrix {
  public:
    SubLTMatrix (LTMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubLTMatrix (const SubLTMatrix&) throw (std::bad_alloc);
    ~SubLTMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubLTMatrix& operator= (const SubLTMatrix&) throw (std::invalid_argument);
  private:
    LTMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
  };

  class SubReUTMatrix : public ReMatrix {
  public:
    SubReUTMatrix (UTMatrix&, const IndicatorArray&, const IndicatorArray&, bool =true, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg4 and arg3.entry(j)=arg5
    // (the upper right rectangle of an upper triangular matrix)    
    SubReUTMatrix (const SubReUTMatrix&) throw (std::bad_alloc);
    ~SubReUTMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubReUTMatrix& operator= (const SubReUTMatrix&) throw (std::invalid_argument);
  private:
    UTMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int* j_ind;
    unsigned int i_size;
    unsigned int j_size;
  };

  class SubUTMatrix : public UTMatrix {
  public:
    SubUTMatrix (UTMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubUTMatrix (const SubUTMatrix&) throw (std::bad_alloc);
    ~SubUTMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubUTMatrix& operator= (const SubUTMatrix&) throw (std::invalid_argument);
  private:
    UTMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
  };

  class SubDiMatrix : public DiMatrix {
  public:
    SubDiMatrix (DiMatrix&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new submatrix from the matrix (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i,j) for which arg2.entry(i)=arg3 and arg2.entry(j)=arg3
    SubDiMatrix (const SubDiMatrix&) throw (std::bad_alloc);
    ~SubDiMatrix () throw ();
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubDiMatrix& operator= (const SubDiMatrix&) throw (std::invalid_argument);
  private:
    DiMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
  };

  class SubColumnVector : public ColumnVector {
  public:
    SubColumnVector (ColumnVector&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new subvector from the vector (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i) for which arg2.entry(i)=arg3
    SubColumnVector (ReMatrix&, const IndicatorArray&, unsigned int, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new subvector from the matrix (arg1); arg3 is the column index
    SubColumnVector (ReMatrix&, unsigned int) throw (std::invalid_argument, std::bad_alloc);
    // create a new subvector from the matrix (arg1); arg2 is the column index
    SubColumnVector (const SubColumnVector&) throw (std::bad_alloc);
    ~SubColumnVector () throw ();
    double& r_entry (unsigned int) throw ();
    double entry (unsigned int) const throw ();
    double& r_at (unsigned int) throw (std::out_of_range);
    double at (unsigned int) const throw (std::out_of_range);
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubColumnVector& operator= (const SubColumnVector&) throw (std::invalid_argument);
    void to_c_array (double*) const throw ();
    void from_c_array (const double*) throw ();
  private:
    ReMatrix& complete_matrix;
    unsigned int* i_ind;
    unsigned int i_size;
    const unsigned int j;
  };

  class SubRowVector : public RowVector {
  public:
    SubRowVector (RowVector&, const IndicatorArray&, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new subvector from the vector (arg1) according
    // to the indicator array (arg2): the submatrix contains only these
    // entries (i) for which arg2.entry(i)=arg3
    SubRowVector (ReMatrix&, const IndicatorArray&, unsigned int, bool =true) throw (std::invalid_argument, std::bad_alloc);
    // create a new subvector from a matrix (arg1); arg3 is the row index
    SubRowVector (ReMatrix&, unsigned int) throw (std::invalid_argument, std::bad_alloc);
    // create a new subvector from a matrix (arg1); arg2 is the row index
    SubRowVector (const SubRowVector&) throw (std::bad_alloc);
    ~SubRowVector () throw ();
    double& r_entry (unsigned int) throw ();
    double entry (unsigned int) const throw ();
    double& r_at (unsigned int) throw (std::out_of_range);
    double at (unsigned int) const throw (std::out_of_range);
    double& r_entry (unsigned int, unsigned int) throw ();
    double entry (unsigned int, unsigned int) const throw ();
    double& r_at (unsigned int, unsigned int) throw (std::out_of_range);
    double at (unsigned int, unsigned int) const throw (std::out_of_range);
    unsigned int number_columns () const throw ();
    unsigned int number_rows () const throw ();
    void clear () throw ();
    virtual const SubRowVector& operator= (const SubRowVector&) throw (std::invalid_argument);
    void to_c_array (double*) const throw ();
    void from_c_array (const double*) throw ();
  private:
    ReMatrix& complete_matrix;
    unsigned int* j_ind;
    unsigned int j_size;
    const unsigned int i;
  };

}


#endif
