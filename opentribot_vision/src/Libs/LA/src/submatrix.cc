
#include "submatrix.h"
#include <algorithm>

using namespace Martin;
using namespace std;


// -----------------------------------------------------------
// class SubMatrix:

SubMatrix::SubMatrix (const Matrix& m, const IndicatorArray& ind1, const IndicatorArray& ind2, bool b1, bool b2) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows() || ind2.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubMatrix::SubMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  j_size=ind2.sub_size (b2);
  if (i_size==0 || j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubMatrix::SubMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  unsigned int p=0;
  const unsigned int ind1_size = ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
  p=0;
  const unsigned int ind2_size = ind2.size();
  for (unsigned int j=1; j<=ind2_size; j++)
    if (ind2.entry(j)==b2) {
      j_ind [p]=j;
      p++;
    }
}
  
SubMatrix::SubMatrix (const SubMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  j_size = m.j_size;
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
  for (unsigned int j=0; j<j_size; j++)
    j_ind [j]=m.j_ind [j];
}

SubMatrix::~SubMatrix () throw () {
  delete [] i_ind;
  delete [] j_ind;
}

double SubMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], j_ind [j-1]);
}

double SubMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubMatrix::at");
  return entry (i,j);
}

unsigned int SubMatrix::number_columns () const throw () {
  return j_size;
}

unsigned int SubMatrix::number_rows () const throw () {
  return i_size;
}

// -----------------------------------------------------------
// class SubSymmetricMatrix:

SubSymmetricMatrix::SubSymmetricMatrix (const SymmetricMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubSymmetricMatrix::SubSymmetricMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubSymmetricMatrix::SubSymmetricMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}
  
SubSymmetricMatrix::SubSymmetricMatrix (const SubSymmetricMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubSymmetricMatrix::~SubSymmetricMatrix () throw () {
  delete [] i_ind;
}

double SubSymmetricMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double SubSymmetricMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubSymmetricMatrix::at");
  return entry (i,j);
}

unsigned int SubSymmetricMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubSymmetricMatrix::number_rows () const throw () {
  return i_size;
}

// -----------------------------------------------------------
// class SubUpperTriangularMatrix:

SubUpperTriangularMatrix::SubUpperTriangularMatrix (const UpperTriangularMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubUpperTriangularMatrix::SubUpperTriangularMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubUpperTriangularMatrix::SubUpperTriangularMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}
  
SubUpperTriangularMatrix::SubUpperTriangularMatrix (const SubUpperTriangularMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubUpperTriangularMatrix::~SubUpperTriangularMatrix () throw () {
  delete [] i_ind;
}

double SubUpperTriangularMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double SubUpperTriangularMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubUpperTriangularMatrix::at");
  return entry (i,j);
}

unsigned int SubUpperTriangularMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubUpperTriangularMatrix::number_rows () const throw () {
  return i_size;
}

// -----------------------------------------------------------
// class SubLowerTriangularMatrix:

SubLowerTriangularMatrix::SubLowerTriangularMatrix (const LowerTriangularMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubLowerTriangularMatrix::SubLowerTriangularMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubLowerTriangularMatrix::SubLowerTriangularMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}
  
SubLowerTriangularMatrix::SubLowerTriangularMatrix (const SubLowerTriangularMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubLowerTriangularMatrix::~SubLowerTriangularMatrix () throw () {
  delete [] i_ind;
}

double SubLowerTriangularMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double SubLowerTriangularMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubLowerTriangularMatrix::at");
  return entry (i,j);
}

unsigned int SubLowerTriangularMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubLowerTriangularMatrix::number_rows () const throw () {
  return i_size;
}

// -----------------------------------------------------------
// class SubDiagonalMatrix:

SubDiagonalMatrix::SubDiagonalMatrix (const DiagonalMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubDiagonalMatrix::SubDiagonalMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubDiagonalMatrix::SubDiagonalMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}
  
SubDiagonalMatrix::SubDiagonalMatrix (const SubDiagonalMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubDiagonalMatrix::~SubDiagonalMatrix () throw () {
  delete [] i_ind;
}

double SubDiagonalMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double SubDiagonalMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubDiagonalMatrix::at");
  return entry (i,j);
}

unsigned int SubDiagonalMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubDiagonalMatrix::number_rows () const throw () {
  return i_size;
}





// =======================================================================
// class SubReMatrix:


SubReMatrix::SubReMatrix (ReMatrix& m, const IndicatorArray& ind1, const IndicatorArray& ind2, bool b1, bool b2) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows() || ind2.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubReMatrix::SubReMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  j_size=ind2.sub_size (b2);
  if (i_size==0 || j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubReMatrix::SubReMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
  p=0;
  const unsigned int ind2_size=ind2.size();
  for (unsigned int j=1; j<=ind2_size; j++)
    if (ind2.entry(j)==b2) {
      j_ind [p]=j;
      p++;
    }
}
  
SubReMatrix::SubReMatrix (const SubReMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  j_size = m.j_size;
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
  for (unsigned int j=0; j<j_size; j++)
    j_ind [j]=m.j_ind [j];
}

SubReMatrix::~SubReMatrix () throw () {
  delete [] i_ind;
  delete [] j_ind;
}


double& SubReMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], j_ind [j-1]);
}

double SubReMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], j_ind [j-1]);
}

double& SubReMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReMatrix::r_at");
  return r_entry (i,j);
}

double SubReMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReMatrix::at");
  return entry (i,j);
}

unsigned int SubReMatrix::number_columns () const throw () {
  return j_size;
}

unsigned int SubReMatrix::number_rows () const throw () {
  return i_size;
}

void SubReMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=0; j<j_size; j++)
      complete_matrix.r_entry(i_ind [i], j_ind [j])=0;
}

const SubReMatrix& SubReMatrix::operator= (const SubReMatrix& src) throw (invalid_argument) {
  const ReMatrix& src2 (src);
  ReMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}

// -----------------------------------------------------------
// class SubReSyMatrix:

SubReSyMatrix::SubReSyMatrix (SyMatrix& m, const IndicatorArray& ind1, const IndicatorArray& ind2, bool b1, bool b2) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows() || ind2.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubReSyMatrix::SubReSyMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  j_size=ind2.sub_size (b2);
  unsigned int n=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind2.entry(i)==b2)
      if (ind1.entry(i)==b1) 
	n++;
  if (i_size==0 || j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubReSyMatrix::SubReSyMatrix");
  if (n>=2)
    throw invalid_argument ("forbidden combination of indicator arrays in Martin::SubReSyMatrix::SubReSyMatrix");
    
  // create pointer lists
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  unsigned int p1=0;
  unsigned int p2=0;
  for (unsigned int i=1; i<=ind1_size; i++) {
    if (ind1.entry(i)==b1) {
      i_ind [p1]=i;
      p1++;
    } 
    if (ind2.entry(i)==b2) {
      j_ind [p2]=i;
      p2++;
    }
  }
}

SubReSyMatrix::SubReSyMatrix (SyMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubReSyMatrix::SubReSyMatrix");
  // determine size of submatrix
  j_size=ind1.sub_size (b1);
  i_size=ind1.size()-j_size;
  if (i_size==0 || j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubReSyMatrix::SubReSyMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  unsigned int p1=0;
  unsigned int p2=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      j_ind [p2]=i;
      p2++;
    } else {
      i_ind [p1]=i;
      p1++;
    }
}
  
SubReSyMatrix::SubReSyMatrix (const SubReSyMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  j_size = m.j_size;
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
  for (unsigned int j=0; j<j_size; j++)
    j_ind [j]=m.j_ind [j];
}

SubReSyMatrix::~SubReSyMatrix () throw () {
  delete [] i_ind;
  delete [] j_ind;
}


double& SubReSyMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], j_ind [j-1]);
}

double SubReSyMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], j_ind [j-1]);
}

double& SubReSyMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReSyMatrix::r_at");
  return r_entry (i,j);
}

double SubReSyMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReSyMatrix::at");
  return entry (i,j);
}

unsigned int SubReSyMatrix::number_columns () const throw () {
  return j_size;
}

unsigned int SubReSyMatrix::number_rows () const throw () {
  return i_size;
}

void SubReSyMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=0; j<j_size; j++)
      complete_matrix.r_entry(i_ind [i], j_ind [j])=0;
}

const SubReSyMatrix& SubReSyMatrix::operator= (const SubReSyMatrix& src) throw (invalid_argument) {
  const ReMatrix& src2 (src);
  ReMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// -------------------------------
// class SubSyMatrix:

SubSyMatrix::SubSyMatrix (SyMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubSyMatrix::SubSyMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubSyMatrix::SubSyMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}

SubSyMatrix::SubSyMatrix (const SubSyMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubSyMatrix::~SubSyMatrix () throw () {
  delete [] i_ind;
}

double& SubSyMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], i_ind [j-1]);
}

double SubSyMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double& SubSyMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::r_at");
  return r_entry (i,j);
}

double SubSyMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::at");
  return entry (i,j);
}

unsigned int SubSyMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubSyMatrix::number_rows () const throw () {
  return i_size;
}

void SubSyMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=0; j<=i; j++)
      complete_matrix.r_entry(i_ind [i], i_ind [j])=0;
}

const SubSyMatrix& SubSyMatrix::operator= (const SubSyMatrix& src) throw (invalid_argument) {
  const SyMatrix& src2 (src);
  SyMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// -----------------------------------------------------------
// class SubReLTMatrix:

SubReLTMatrix::SubReLTMatrix (LTMatrix& m, const IndicatorArray& ind1, const IndicatorArray& ind2, bool b1, bool b2) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows() || ind2.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubReLTMatrix::SubReLTMatrix");
  // determine size of submatrix
  i_size=0;
  j_size=0;
  unsigned int max_j=0;
  unsigned int min_i=m.number_rows()+1;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++) {
    if (ind1.entry(i)==b1) {
      i_size++;
      min_i=min (i, min_i);
    }
    if (ind2.entry(i)==b2) {
      j_size++;
      max_j=max (i, max_j); 
    }
  }
  if (i_size==0 || j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubReLTMatrix::SubReLTMatrix");
  if (max_j>min_i)
    throw invalid_argument ("forbidden combination of indicator arrays in Martin::SubReLTMatrix::SubReLTMatrix");
    
  // create pointer lists
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  unsigned int p1=0;
  unsigned int p2=0;
  for (unsigned int i=1; i<=ind1_size; i++) {
    if (ind1.entry(i)==b1) {
      i_ind [p1]=i;
      p1++;
    } 
    if (ind2.entry(i)==b2) {
      j_ind [p2]=i;
      p2++;
    }
  }
}

SubReLTMatrix::SubReLTMatrix (const SubReLTMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  j_size = m.j_size;
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
  for (unsigned int j=0; j<j_size; j++)
    j_ind [j]=m.j_ind [j];
}

SubReLTMatrix::~SubReLTMatrix () throw () {
  delete [] i_ind;
  delete [] j_ind;
}


double& SubReLTMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], j_ind [j-1]);
}

double SubReLTMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], j_ind [j-1]);
}

double& SubReLTMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReLTMatrix::r_at");
  return r_entry (i,j);
}

double SubReLTMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReLTMatrix::at");
  return entry (i,j);
}

unsigned int SubReLTMatrix::number_columns () const throw () {
  return j_size;
}

unsigned int SubReLTMatrix::number_rows () const throw () {
  return i_size;
}

void SubReLTMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=0; j<j_size; j++)
      complete_matrix.r_entry(i_ind [i], j_ind [j])=0;
}

const SubReLTMatrix& SubReLTMatrix::operator= (const SubReLTMatrix& src) throw (invalid_argument) {
  const ReMatrix& src2 (src);
  ReMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// -------------------------------
// class SubLTMatrix:

SubLTMatrix::SubLTMatrix (LTMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubLTMatrix::SubLTMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubLTMatrix::SubLTMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}

SubLTMatrix::SubLTMatrix (const SubLTMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubLTMatrix::~SubLTMatrix () throw () {
  delete [] i_ind;
}

double& SubLTMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], i_ind [j-1]);
}

double SubLTMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double& SubLTMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::r_at");
  return r_entry (i,j);
}

double SubLTMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::at");
  return entry (i,j);
}

unsigned int SubLTMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubLTMatrix::number_rows () const throw () {
  return i_size;
}

void SubLTMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=0; j<=i; j++)
      complete_matrix.r_entry(i_ind [i], i_ind [j])=0;
}

const SubLTMatrix& SubLTMatrix::operator= (const SubLTMatrix& src) throw (invalid_argument) {
  const LTMatrix& src2 (src);
  LTMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// -----------------------------------------------------------
// class SubReUTMatrix:

SubReUTMatrix::SubReUTMatrix (UTMatrix& m, const IndicatorArray& ind1, const IndicatorArray& ind2, bool b1, bool b2) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows() || ind2.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubReUTMatrix::SubReUTMatrix");
  // determine size of submatrix
  i_size=0;
  j_size=0;
  unsigned int min_j=m.number_columns()+1;
  unsigned int max_i=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++) {
    if (ind1.entry(i)==b1) {
      i_size++;
      max_i=max (i, max_i);
    }
    if (ind2.entry(i)==b2) {
      j_size++;
      min_j=min (i, min_j); 
    }
  }
  if (i_size==0 || j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubReUTMatrix::SubReUTMatrix");
  if (max_i>min_j)
    throw invalid_argument ("forbidden combination of indicator arrays in Martin::SubReUTMatrix::SubReUTMatrix");
    
  // create pointer lists
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  unsigned int p1=0;
  unsigned int p2=0;
  for (unsigned int i=1; i<=ind1_size; i++) {
    if (ind1.entry(i)==b1) {
      i_ind [p1]=i;
      p1++;
    } 
    if (ind2.entry(i)==b2) {
      j_ind [p2]=i;
      p2++;
    }
  }
}

SubReUTMatrix::SubReUTMatrix (const SubReUTMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  j_size = m.j_size;
  i_ind = new unsigned int [i_size];
  j_ind = new unsigned int [j_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
  for (unsigned int j=0; j<j_size; j++)
    j_ind [j]=m.j_ind [j];
}

SubReUTMatrix::~SubReUTMatrix () throw () {
  delete [] i_ind;
  delete [] j_ind;
}


double& SubReUTMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], j_ind [j-1]);
}

double SubReUTMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], j_ind [j-1]);
}

double& SubReUTMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReUTMatrix::r_at");
  return r_entry (i,j);
}

double SubReUTMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>j_size)
    throw out_of_range ("index out of range in Martin::SubReUTMatrix::at");
  return entry (i,j);
}

unsigned int SubReUTMatrix::number_columns () const throw () {
  return j_size;
}

unsigned int SubReUTMatrix::number_rows () const throw () {
  return i_size;
}

void SubReUTMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=0; j<j_size; j++)
      complete_matrix.r_entry(i_ind [i], j_ind [j])=0;
}

const SubReUTMatrix& SubReUTMatrix::operator= (const SubReUTMatrix& src) throw (invalid_argument) {
  const ReMatrix& src2 (src);
  ReMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// -------------------------------
// class SubUTMatrix:

SubUTMatrix::SubUTMatrix (UTMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubUTMatrix::SubUTMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubUTMatrix::SubUTMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}

SubUTMatrix::SubUTMatrix (const SubUTMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubUTMatrix::~SubUTMatrix () throw () {
  delete [] i_ind;
}

double& SubUTMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], i_ind [j-1]);
}

double SubUTMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double& SubUTMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || j<1 || j>i_size || i>j)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::r_at");
  return r_entry (i,j);
}

double SubUTMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::at");
  return entry (i,j);
}

unsigned int SubUTMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubUTMatrix::number_rows () const throw () {
  return i_size;
}

void SubUTMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    for (unsigned int j=i; j<=i_size; j++)
      complete_matrix.r_entry(i_ind [i], i_ind [j])=0;
}

const SubUTMatrix& SubUTMatrix::operator= (const SubUTMatrix& src) throw (invalid_argument) {
  const UTMatrix& src2 (src);
  UTMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// -------------------------------
// class SubDiMatrix:

SubDiMatrix::SubDiMatrix (DiMatrix& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubDiMatrix::SubDiMatrix");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubDiMatrix::SubDiMatrix");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}

SubDiMatrix::SubDiMatrix (const SubDiMatrix& m) throw (bad_alloc) : complete_matrix (m.complete_matrix) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubDiMatrix::~SubDiMatrix () throw () {
  delete [] i_ind;
}

double& SubDiMatrix::r_entry (unsigned int i, unsigned int j) throw () {
  return complete_matrix.r_entry (i_ind [i-1], i_ind [j-1]);
}

double SubDiMatrix::entry (unsigned int i, unsigned int j) const throw () {
  return complete_matrix.entry (i_ind [i-1], i_ind [j-1]);
}

double& SubDiMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) {
  if (i<1 || i>i_size || i!=j)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::r_at");
  return r_entry (i,j);
}

double SubDiMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) {
  if (i<1 || j<1 || i>i_size || j>i_size)
    throw out_of_range ("index out of range in Martin::SubSyMatrix::at");
  return entry (i,j);
}

unsigned int SubDiMatrix::number_columns () const throw () {
  return i_size;
}

unsigned int SubDiMatrix::number_rows () const throw () {
  return i_size;
}

void SubDiMatrix::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    complete_matrix.r_entry(i_ind [i], i_ind [i])=0;
}

const SubDiMatrix& SubDiMatrix::operator= (const SubDiMatrix& src) throw (invalid_argument) {
  const DiMatrix& src2 (src);
  DiMatrix& this2 (*this);
  this2.operator= (src2);
  return (*this);
}


// ------------------------------------------------------------
// class SubColumnVector:


SubColumnVector::SubColumnVector (ColumnVector& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m), j(1) {
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubColumnVector::SubColumnVector");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubColumnVector::SubColumnVector");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int i=1; i<=ind1_size; i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}

SubColumnVector::SubColumnVector (ReMatrix& m, unsigned int jj) throw (invalid_argument, bad_alloc) : complete_matrix (m), j(jj) {
  if (j<1 || j>m.number_columns())
    throw invalid_argument ("column index exceeds size of matrix in Martin::SubColumnVector::SubColumnVector");
  i_size=m.number_rows();

  // create pointer lists
  i_ind = new unsigned int [i_size];
  for (unsigned int i=1; i<=i_size; i++)
    i_ind [i-1]=i;
}

SubColumnVector::SubColumnVector (ReMatrix& m, const IndicatorArray& ind1, unsigned int jj, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m), j(jj) {
  if (j<1 || j>m.number_columns())
    throw invalid_argument ("column index exceeds size of matrix in Martin::SubColumnVector::SubColumnVector");
  if (ind1.size()!=m.number_rows())
    throw invalid_argument ("dimension mismatch in Martin::SubColumnVector::SubColumnVector");
  // determine size of submatrix
  i_size=ind1.sub_size (b1);
  if (i_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubColumnVector::SubColumnVector");

  // create pointer lists
  i_ind = new unsigned int [i_size];
  unsigned int p=0;
  for (unsigned int i=1; i<=ind1.size(); i++)
    if (ind1.entry(i)==b1) {
      i_ind [p]=i;
      p++;
    }
}

SubColumnVector::SubColumnVector (const SubColumnVector& m) throw (bad_alloc) : complete_matrix (m.complete_matrix), j (m.j) {
  i_size = m.i_size;
  i_ind = new unsigned int [i_size];
  for (unsigned int i=0; i<i_size; i++)
    i_ind [i]=m.i_ind [i];
}

SubColumnVector::~SubColumnVector () throw () {
  delete [] i_ind;
}

double& SubColumnVector::r_entry (unsigned int i) throw () {
  return complete_matrix.r_entry (i_ind [i-1],j);
}

double SubColumnVector::entry (unsigned int i) const throw () {
  return complete_matrix.entry (i_ind [i-1],j);
}

double& SubColumnVector::r_at (unsigned int i) throw (out_of_range) {
  if (i<1 || i>i_size)
    throw out_of_range ("index out of range in Martin::ColumnVector::r_at");
  return r_entry (i);
}

double SubColumnVector::at (unsigned int i) const throw (out_of_range) {
  if (i<1 || i>i_size)
    throw out_of_range ("index out of range in Martin::ColumnVector::at");
  return entry (i);
}

unsigned int SubColumnVector::number_columns () const throw () {
  return 1;
}

unsigned int SubColumnVector::number_rows () const throw () {
  return i_size;
}

void SubColumnVector::clear () throw () {
  for (unsigned int i=0; i<i_size; i++)
    complete_matrix.r_entry(i_ind [i],j)=0;
}
  

double& SubColumnVector::r_entry (unsigned int i, unsigned int) throw () {
  return r_entry (i);
}

double SubColumnVector::entry (unsigned int i, unsigned int) const throw () {
  return entry (i);
}

double& SubColumnVector::r_at (unsigned int i, unsigned int jj) throw (out_of_range) {
  if (i<1 || i>i_size || jj!=1)
    throw out_of_range ("index out of range in Martin::SubColumnVector::r_at");
  return r_entry (i);
}

double SubColumnVector::at (unsigned int i, unsigned int jj) const throw (out_of_range) {
  if (i<1 || i>i_size || jj!=1)
    throw out_of_range ("index out of range in Martin::SubColumnVector::at");
  return entry (i);
}

const SubColumnVector& SubColumnVector::operator= (const SubColumnVector& src) throw (invalid_argument) {
  const ColumnVector& src2 (src);
  ColumnVector& this2 (*this);
  this2.operator= (src2);
  return (*this);
}

void SubColumnVector::to_c_array (double* g) const throw () {
  const unsigned int num_row=number_rows();
  for (unsigned int i=1; i<=num_row; i++) {
    *g = entry(i);
    g++;
  }
}

void SubColumnVector::from_c_array (const double* g) throw () {
  const unsigned int num_row=number_rows();
  for (unsigned int i=1; i<=num_row; i++) {
    r_entry(i)=*g;
    g++;
  }
}


// ------------------------------------------------------------
// class SubRowVector:


SubRowVector::SubRowVector (RowVector& m, const IndicatorArray& ind1, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m), i(1) {
  if (ind1.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubRowVector::SubRowVector");
  // determine size of submatrix
  j_size=ind1.sub_size (b1);
  if (j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubRowVector::SubRowVector");

  // create pointer lists
  j_ind = new unsigned int [j_size];
  unsigned int p=0;
  const unsigned int ind1_size=ind1.size();
  for (unsigned int j=1; j<=ind1_size; j++)
    if (ind1.entry(j)==b1) {
      j_ind [p]=j;
      p++;
    }
}

SubRowVector::SubRowVector (ReMatrix& m, const IndicatorArray& ind1, unsigned int ii, bool b1) throw (invalid_argument, bad_alloc) : complete_matrix (m), i(ii) {
  if (i<1 || i>m.number_rows())
    throw invalid_argument ("row index exceeds size of matrix in Martin::SubRowVector::SubRowVector");
  if (ind1.size()!=m.number_columns())
    throw invalid_argument ("dimension mismatch in Martin::SubRowVector::SubRowVector");
  // determine size of submatrix
  j_size=ind1.sub_size (b1);
  if (j_size==0)
    throw invalid_argument ("submatrix of zero dimension in Martin::SubRowVector::SubRowVector");

  // create pointer lists
  j_ind = new unsigned int [j_size];
  unsigned int p=0;
  for (unsigned int j=1; j<=ind1.size(); j++)
    if (ind1.entry(j)==b1) {
      j_ind [p]=j;
      p++;
    }
}

SubRowVector::SubRowVector (ReMatrix& m, unsigned int ii) throw (invalid_argument, bad_alloc) : complete_matrix (m), i(ii) {
  if (i<1 || i>m.number_rows())
    throw invalid_argument ("row index exceeds size of matrix in Martin::SubRowVector::SubRowVector");
  // determine size of submatrix
  j_size=m.number_columns();

  // create pointer lists
  j_ind = new unsigned int [j_size];
  for (unsigned int j=1; j<=j_size; j++)
    j_ind [j-1]=j;
}

SubRowVector::SubRowVector (const SubRowVector& m) throw (bad_alloc) : complete_matrix (m.complete_matrix), i (m.i) {
  j_size = m.j_size;
  j_ind = new unsigned int [j_size];
  for (unsigned int j=0; j<j_size; j++)
    j_ind [j]=m.j_ind [j];
}

SubRowVector::~SubRowVector () throw () {
  delete [] j_ind;
}

double& SubRowVector::r_entry (unsigned int j) throw () {
  return complete_matrix.r_entry (i, j_ind [j-1]);
}

double SubRowVector::entry (unsigned int j) const throw () {
  return complete_matrix.entry (i, j_ind [j-1]);
}

double& SubRowVector::r_at (unsigned int j) throw (out_of_range) {
  if (j<1 || j>j_size)
    throw out_of_range ("index out of range in Martin::RowVector::r_at");
  return r_entry (j);
}

double SubRowVector::at (unsigned int j) const throw (out_of_range) {
  if (j<1 || j>j_size)
    throw out_of_range ("index out of range in Martin::RowVector::at");
  return entry (j);
}

unsigned int SubRowVector::number_columns () const throw () {
  return j_size;
}

unsigned int SubRowVector::number_rows () const throw () {
  return 1;
}

void SubRowVector::clear () throw () {
  for (unsigned int j=0; j<j_size; j++)
    complete_matrix.r_entry(i, j_ind [j])=0;
}
  

double& SubRowVector::r_entry (unsigned int, unsigned int j) throw () {
  return r_entry (j);
}

double SubRowVector::entry (unsigned int, unsigned int j) const throw () {
  return entry (j);
}

double& SubRowVector::r_at (unsigned int ii, unsigned int j) throw (out_of_range) {
  if (j<1 || j>j_size || ii!=1)
    throw out_of_range ("index out of range in Martin::RowVector::r_at");
  return r_entry (j);
}

double SubRowVector::at (unsigned int ii, unsigned int j) const throw (out_of_range) {
  if (j<1 || j>j_size || ii!=1)
    throw out_of_range ("index out of range in Martin::RowVector::at");
  return entry (j);
}

const SubRowVector& SubRowVector::operator= (const SubRowVector& src) throw (invalid_argument) {
  const RowVector& src2 (src);
  RowVector& this2 (*this);
  this2.operator= (src2);
  return (*this);
}

void SubRowVector::to_c_array (double* g) const throw () {
  const unsigned int num_col=number_columns();
  for (unsigned int i=1; i<=num_col; i++) {
    *g = entry(i);
    g++;
  }
}

void SubRowVector::from_c_array (const double* g) throw () {
  const unsigned int num_col=number_columns();
  for (unsigned int i=1; i<=num_col; i++) {
    r_entry(i)=*g;
    g++;
  }
}

