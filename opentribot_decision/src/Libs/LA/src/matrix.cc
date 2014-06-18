
#include "matrix.h"

using namespace Martin;
using namespace std;


ReMatrix::ReMatrix (unsigned int i, unsigned int j) throw (bad_alloc, invalid_argument) : f (new double [i*j]), nr (i), nc (j) { 
  if (i<1 || j<1) 
    throw invalid_argument("invalid matrix dimension in Martin::ReMatrix::ReMatrix"); 
; 
}
ReMatrix::ReMatrix () throw () : f(NULL), nr (0), nc(0) { ; }
ReMatrix::ReMatrix (const ReMatrix& m) throw (bad_alloc) : f (new double [m.number_rows()*m.number_columns()]), nr (m.number_rows ()), nc (m.number_columns ()) { 
  for (unsigned int i=1; i<=nr; i++)
    for (unsigned int j=1; j<=nc; j++)
      r_entry (i,j)=m.entry (i,j);
}
ReMatrix::ReMatrix (const Matrix& m) throw (bad_alloc) : f (new double [m.number_rows()*m.number_columns()]), nr (m.number_rows ()), nc (m.number_columns ()) { 
  for (unsigned int i=1; i<=nr; i++)
    for (unsigned int j=1; j<=nc; j++)
      r_entry (i,j)=m.entry (i,j);
}
ReMatrix::~ReMatrix () throw () {
  if (f!=NULL)
    delete [] f;
}
unsigned int ReMatrix::number_columns () const throw () { return nc; }
unsigned int ReMatrix::number_rows () const throw () { return nr; }
double ReMatrix::entry (unsigned int i, unsigned int j) const throw () { return f [j-1+(i-1)*nc]; }
double& ReMatrix::r_entry (unsigned int i, unsigned int j) throw () { return f [j-1+(i-1)*nc]; }
double ReMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) { 
  if (i<1 || i>nr || j<1 || j>nc)
    throw out_of_range ("matrix index out of range in Martin::ReMatrix::at");
  return entry (i,j); 
}
double& ReMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) { 
  if (i<1 || i>nr || j<1 || j>nc)
    throw out_of_range ("matrix index out of range in Martin::ReMatrix::r_at");
  return r_entry (i,j); 
}
void ReMatrix::clear () throw () { 
  for (unsigned int i=0; i<nr*nc; i++) 
    f [i]=0;
}
const ReMatrix& ReMatrix::operator= (const Matrix& src) throw (invalid_argument) {
  if (src.number_rows()!=number_rows() || src.number_columns()!=number_columns())
    throw invalid_argument ("mismatch in matrix size in Martin::ReMatrix::operator=");
  for (unsigned int i=1; i<=number_rows(); i++)
    for (unsigned int j=1; j<=number_columns(); j++)
      r_entry (i,j)=src.entry(i,j);
  return (*this);
}
const ReMatrix& ReMatrix::operator= (const ReMatrix& src) throw (invalid_argument) {
  const Matrix& src2 (src);
  return operator= (src2);
}
void ReMatrix::resize (unsigned int r, unsigned int c) throw (std::bad_alloc) {
  double* g = new double [r*c];
  delete [] f;
  f=g;
  nr=r;
  nc=c;
}

ColumnVector::ColumnVector (unsigned int i) throw (bad_alloc, invalid_argument) : ReMatrix (i,1) { ; }
ColumnVector::ColumnVector () throw () { ; }
ColumnVector::ColumnVector (const ColumnVector& m) throw (bad_alloc) : ReMatrix (m) { ; }
ColumnVector::ColumnVector (const Matrix& m) throw (bad_alloc, invalid_argument) : ReMatrix (m) { 
  if (m.number_columns()!=1)
    throw invalid_argument ("invalid initializing matrix in Martin::ColumnVector::ColumnVector");
}
ColumnVector::ColumnVector (const double* g, unsigned int n) throw (bad_alloc, invalid_argument) : ReMatrix (n,1) {
  for (unsigned int i=0; i<n; i++)
    f[i] = g[i];
}
void ColumnVector::resize (unsigned int d) throw (std::bad_alloc) {
  ReMatrix* rm = this;
  rm->resize (d,1);
}
double ColumnVector::entry (unsigned int i) const throw () { return f[i-1]; } // not nice but efficient
double& ColumnVector::r_entry (unsigned int i) throw () { return f[i-1]; } // not nice but efficient
double ColumnVector::at (unsigned int i) const throw (out_of_range) { 
  if (i<1 || i>number_rows())
    throw out_of_range ("matrix index out of range in Martin::ColumnVector::at");
  return ReMatrix::entry (i,1); 
}
double& ColumnVector::r_at (unsigned int i) throw (out_of_range) { 
  if (i<1 || i>number_rows())
    throw out_of_range ("matrix index out of range in Martin::ColumnVector::r_at");
  return ReMatrix::r_entry (i,1); 
}
const ColumnVector& ColumnVector::operator= (const ColumnVector& src) throw (invalid_argument) {
  if (src.number_rows()!=number_rows())
    throw invalid_argument ("mismatch in matrix size in Martin::ColumnVector::operator=");
  for (unsigned int i=1; i<=number_rows(); i++)
    r_entry (i)=src.entry(i);
  return (*this);
}
void ColumnVector::to_c_array (double* g) const throw () {
  for (unsigned int i=0; i<nr; i++)
    g[i] = f[i];
}
void ColumnVector::from_c_array (const double* g) throw () {
  for (unsigned int i=0; i<nr; i++)
    f[i] = g[i];
}

RowVector::RowVector (unsigned int i) throw (bad_alloc, invalid_argument) : ReMatrix (1,i) { ; }
RowVector::RowVector () throw () { ; }
RowVector::RowVector (const RowVector& m) throw (bad_alloc) : ReMatrix (m) { ; }
RowVector::RowVector (const Matrix& m) throw (bad_alloc, invalid_argument) : ReMatrix (m) { 
  if (m.number_rows()!=1)
    throw invalid_argument ("invalid initializing matrix in Martin::RowVector::RowVector");
}
void RowVector::resize (unsigned int d) throw (std::bad_alloc) {
  ReMatrix* rm = this;
  rm->resize (1,d);
}
RowVector::RowVector (const double* g, unsigned int n) throw (bad_alloc, invalid_argument) : ReMatrix (1,n) {
  for (unsigned int i=0; i<n; i++)
    f[i] = g[i];
}
double RowVector::entry (unsigned int i) const throw () { return f[i-1]; } // not nice but efficient
double& RowVector::r_entry (unsigned int i) throw () { return f[i-1]; } // not nice but efficient
double RowVector::at (unsigned int i) const throw (out_of_range) { 
  if (i<1 || i>number_columns())
    throw out_of_range ("matrix index out of range in Martin::RowVector::at");
  return ReMatrix::entry (1,i); 
}
double& RowVector::r_at (unsigned int i) throw (out_of_range) { 
  if (i<1 || i>number_columns())
    throw out_of_range ("matrix index out of range in Martin::RowVector::r_at");
  return ReMatrix::r_entry (1,i); 
}
const RowVector& RowVector::operator= (const RowVector& src) throw (invalid_argument) {
  if (src.number_columns()!=number_columns())
    throw invalid_argument ("mismatch in matrix size in Martin::RowVector::operator=");
  for (unsigned int i=1; i<=number_columns(); i++)
    r_entry (i)=src.entry(i);
  return (*this);
}
void RowVector::to_c_array (double* g) const throw () {
  for (unsigned int i=0; i<nc; i++)
    g[i] = f[i];
}
void RowVector::from_c_array (const double* g) throw () {
  for (unsigned int i=0; i<nc; i++)
    f[i] = g[i];
}

SyMatrix::SyMatrix (unsigned int d) throw (bad_alloc, invalid_argument) : f (new double [(d*(d+1))/2]), n(d) { 
  if (d<1)
    throw invalid_argument ("invalid matrix dimensions in Martin::SyMatrix::SyMatrix");
}
SyMatrix::SyMatrix () throw () : f(NULL), n(0) { ; }
SyMatrix::SyMatrix (const SyMatrix& m) throw (bad_alloc) : f (new double [(m.number_rows()*(m.number_rows()+1))/2]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    for (unsigned int j=1; j<=i; j++)
      r_entry (i,j)=m.entry (i,j);
}
SyMatrix::SyMatrix (const SymmetricMatrix& m) throw (bad_alloc) : f (new double [(m.number_rows()*(m.number_rows()+1))/2]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    for (unsigned int j=1; j<=i; j++)
      r_entry (i,j)=m.entry (i,j);
}
SyMatrix::~SyMatrix () throw () { 
  if (f!=NULL)
    delete [] f; 
}
void SyMatrix::resize (unsigned int d) throw (std::bad_alloc) {
  double* g = new double [(d*(d+1))/2];
  delete [] f;
  f=g;
  n=d;
}
unsigned int SyMatrix::number_columns () const throw () { return n; }
unsigned int SyMatrix::number_rows () const throw () { return n; }
double SyMatrix::entry (unsigned int i, unsigned int j) const throw () { return (i>j ? f [(i*(i-1))/2+j-1] : f [(j*(j-1))/2+i-1]); }
double& SyMatrix::r_entry (unsigned int i, unsigned int j) throw () { return (i>j ? f [(i*(i-1))/2+j-1] : f [(j*(j-1))/2+i-1]); }
double SyMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) { 
  if (i<1 || i>n || j<1 || j>n)
    throw out_of_range ("matrix index out of range in Martin::SyMatrix::at");
  return entry (i,j); 
}
double& SyMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) { 
  if (i<1 || i>n || j<1 || j>n)
    throw out_of_range ("matrix index out of range in Martin::SyMatrix::r_at");
  return r_entry (i,j); 
}
void SyMatrix::clear () throw () { 
  for (unsigned int i=0; i<(n*(n+1))/2; i++) 
    f [i]=0;
}
const SyMatrix& SyMatrix::operator= (const SymmetricMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=number_rows())
    throw invalid_argument ("mismatch in matrix size in Martin::SyMatrix::operator=");
  for (unsigned int i=1; i<=number_rows(); i++)
    for (unsigned int j=1; j<=i; j++)
      r_entry (i,j)=src.entry(i,j);
  return (*this);
}
const SyMatrix& SyMatrix::operator= (const SyMatrix& src) throw (invalid_argument) {
  const SymmetricMatrix& src2 (src);
  return operator= (src2);
}

LTMatrix::LTMatrix (unsigned int d) throw (bad_alloc, invalid_argument) : f (new double [(d*(d+1))/2]), n(d) { 
  if (d<1) 
    throw invalid_argument ("invalid matrix dimension in Martin::LTMatrix::LTMatrix");
}
LTMatrix::LTMatrix () throw () : f(NULL), n(0) { ; }
LTMatrix::LTMatrix (const LTMatrix& m) throw (bad_alloc) : f (new double [(m.number_rows()*(m.number_rows()+1))/2]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    for (unsigned int j=1; j<=i; j++)
      r_entry(i,j)=m.entry (i,j);
}
LTMatrix::LTMatrix (const LowerTriangularMatrix& m) throw (bad_alloc) : f (new double [(m.number_rows()*(m.number_rows()+1))/2]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    for (unsigned int j=1; j<=i; j++)
      r_entry(i,j)=m.entry (i,j);
}
LTMatrix::~LTMatrix () throw () { 
  if (f!=NULL)
    delete [] f; 
}
void LTMatrix::resize (unsigned int d) throw (std::bad_alloc) {
  double* g = new double [(d*(d+1))/2];
  delete [] f;
  f=g;
  n=d;
}
unsigned int LTMatrix::number_columns () const throw () { return n; }
unsigned int LTMatrix::number_rows () const throw () { return n; }
double LTMatrix::entry (unsigned int i, unsigned int j) const throw () { return (i>=j ? f [(i*(i-1))/2+j-1] : 0); }
double& LTMatrix::r_entry (unsigned int i, unsigned int j) throw () { return f [(i*(i-1))/2+j-1]; }
double LTMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) { 
  if (i<1 || i>n || j<1 || j>n)
    throw out_of_range ("matrix index out of range in Martin::LTMatrix::at");
  return entry (i,j); 
}
double& LTMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) { 
  if (i<1 || i>n || j<1 || j>i)
    throw out_of_range ("matrix index out of range in Martin::LTMatrix::r_at");
  return r_entry (i,j); 
}
void LTMatrix::clear () throw () { 
  for (unsigned int i=0; i<(n*(n+1))/2; i++) 
    f [i]=0;
}
const LTMatrix& LTMatrix::operator= (const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=number_rows())
    throw invalid_argument ("mismatch in matrix size in Martin::LTMatrix::operator=");
  for (unsigned int i=1; i<=number_rows(); i++)
    for (unsigned int j=1; j<=i; j++)
      r_entry (i,j)=src.entry(i,j);
  return (*this);
}
const LTMatrix& LTMatrix::operator= (const LTMatrix& src) throw (invalid_argument) {
  const LowerTriangularMatrix& src2 (src);
  return operator= (src2);
}

UTMatrix::UTMatrix (unsigned int d) throw (bad_alloc, invalid_argument) : f (new double [(d*(d+1))/2]), n(d) {
  if (d<1)
    throw invalid_argument ("invalid matrix dimension in Martin::UTMatrix::UTMatrix");
}
UTMatrix::UTMatrix () throw () : f(NULL), n(0) { ; }
UTMatrix::UTMatrix (const UTMatrix& m) throw (bad_alloc) : f (new double [(m.number_rows()*(m.number_rows()+1))/2]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    for (unsigned int j=i; j<=n; j++)
      r_entry(i,j)=m.entry (i,j);
}
UTMatrix::UTMatrix (const UpperTriangularMatrix& m) throw (bad_alloc) : f (new double [(m.number_rows()*(m.number_rows()+1))/2]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    for (unsigned int j=i; j<=n; j++)
      r_entry(i,j)=m.entry (i,j);
}
UTMatrix::~UTMatrix () throw () { 
  if (f!=NULL)
    delete [] f; 
}
void UTMatrix::resize (unsigned int d) throw (std::bad_alloc) {
  double* g = new double [(d*(d+1))/2];
  delete [] f;
  f=g;
  n=d;
}
unsigned int UTMatrix::number_columns () const throw () { return n; }
unsigned int UTMatrix::number_rows () const throw () { return n; }
double UTMatrix::entry (unsigned int i, unsigned int j) const throw () { return (i<=j ? f [(j*(j-1))/2+i-1] : 0); }
double& UTMatrix::r_entry (unsigned int i, unsigned int j) throw () { return f [(j*(j-1))/2+i-1]; }
double UTMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) { 
  if (i<1 || i>n || j<1 || j>n)
    throw out_of_range ("matrix index out of range in Martin::UTMatrix::at");
  return entry (i,j); 
}
double& UTMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) { 
  if (i<1 || i>n || j<i || j>n)
    throw out_of_range ("matrix index out of range in Martin::UTMatrix::r_at");
  return r_entry (i,j); 
}
void UTMatrix::clear () throw () { 
  for (unsigned int i=0; i<(n*(n+1))/2; i++) 
    f [i]=0;
}
const UTMatrix& UTMatrix::operator= (const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=number_rows())
    throw invalid_argument ("mismatch in matrix size in Martin::UTMatrix::operator=");
  for (unsigned int i=1; i<=number_rows(); i++)
    for (unsigned int j=i; j<=number_columns(); j++)
      r_entry (i,j)=src.entry(i,j);
  return (*this);
}
const UTMatrix& UTMatrix::operator= (const UTMatrix& src) throw (invalid_argument) {
  const UpperTriangularMatrix& src2 (src);
  return operator= (src2);
}

DiMatrix::DiMatrix (unsigned int d) throw (bad_alloc, invalid_argument) : f (new double [d]), n(d) {
  if (d<1)
    throw invalid_argument ("invalid matrix dimension in Martin::DiMatrix::DiMatrix");
}
DiMatrix::DiMatrix () throw () : f(NULL), n(0) { ; }
DiMatrix::DiMatrix (const DiMatrix& m) throw (bad_alloc) : f (new double [m.number_rows()]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    r_entry (i,i)=m.entry (i,i);
}
DiMatrix::DiMatrix (const DiagonalMatrix& m) throw (bad_alloc) : f (new double [m.number_rows()]), n(m.number_rows()) {
  for (unsigned int i=1; i<=n; i++)
    r_entry (i,i)=m.entry (i,i);
}
DiMatrix::~DiMatrix () throw () { 
  if (f!=NULL)
    delete [] f; 
}
void DiMatrix::resize (unsigned int d) throw (std::bad_alloc) {
  double* g = new double [d];
  delete [] f;
  f=g;
  n=d;
}
unsigned int DiMatrix::number_columns () const throw () { return n; }
unsigned int DiMatrix::number_rows () const throw () { return n; }
double DiMatrix::entry (unsigned int i, unsigned int j) const throw () { return (i==j ? f [i-1] : 0); }
double& DiMatrix::r_entry (unsigned int i, unsigned int) throw () { return f [i-1]; }
double DiMatrix::at (unsigned int i, unsigned int j) const throw (out_of_range) { 
  if (i<1 || i>n || j<1 || j>n)
    throw out_of_range ("matrix index out of range in Martin::DiMatrix::at");
  return entry (i,j); 
}
double& DiMatrix::r_at (unsigned int i, unsigned int j) throw (out_of_range) { 
  if (i<1 || i>n || j!=i)
    throw out_of_range ("matrix index out of range in Martin::DiMatrix::r_at");
  return r_entry (i,j); 
}
void DiMatrix::clear () throw () { 
  for (unsigned int i=0; i<n; i++) 
    f [i]=0;
}
const DiMatrix& DiMatrix::operator= (const DiagonalMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=number_rows())
    throw invalid_argument ("mismatch in matrix size in Martin::DiMatrix::operator=");
  for (unsigned int i=1; i<=number_rows(); i++)
    r_entry (i,i)=src.entry(i,i);
  return (*this);
}
const DiMatrix& DiMatrix::operator= (const DiMatrix& src) throw (invalid_argument) {
  const DiagonalMatrix& src2 (src);
  return operator= (src2);
}


ostream& operator<< (ostream& os, const Martin::Matrix& m) {
  os << '[';
  for (unsigned int i=1; i<m.number_rows(); i++) {
    for (unsigned int j=1; j<m.number_columns(); j++)
      os << m.entry(i,j) << ',';
    os << m.entry (i,m.number_columns()) << ';';
  }
  for (unsigned int j=1; j<m.number_columns(); j++)
    os << m.entry(m.number_rows(),j) << ',';
  os << m.entry (m.number_rows(),m.number_columns()) << ']';
  return os;
} 

