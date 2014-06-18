
#include "matrix_operations.h"
#include <algorithm>
#include <cmath>

using namespace Martin;
using namespace std;


// copy: ------------------------------------------------------------------------------
void Martin::copy (ReMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (dest.number_columns()!=src.number_columns() || dest.number_rows()!=src.number_rows())
    throw invalid_argument("mismatch in matrix dimensions in Martin::copy");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry (i,j)=src.entry(i,j);
    }
  }
}

void Martin::copy (SyMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument) {
  if (dest.number_columns()!=src.number_columns())
    throw invalid_argument("mismatch in matrix dimensions in Martin::copy");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry (i,j)=src.entry(i,j);
    }
  }
}

void Martin::copy (LTMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (dest.number_columns()!=src.number_columns())
    throw invalid_argument("mismatch in matrix dimensions in Martin::copy");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry (i,j)=src.entry(i,j);
    }
  }
}
  
void Martin::copy (UTMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (dest.number_columns()!=src.number_columns())
    throw invalid_argument("mismatch in matrix dimensions in Martin::copy");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry (i,j)=src.entry(i,j);
    }
  }
}

void Martin::copy (DiMatrix& dest, const DiagonalMatrix& src) throw (invalid_argument) {
  if (dest.number_columns()!=src.number_columns())
    throw invalid_argument("mismatch in matrix dimensions in Martin::copy");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry (i,i)=src.entry(i,i);
  }
}


// choleksy-factorization: ------------------------------------------------------------------------------
void Martin::cholesky_factorization (LTMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::cholesky_factorization");
  const unsigned int n=src.number_columns ();
  for (unsigned int i=1; i<=n; i++) {
    double s=0;
    for (unsigned int k=1; k<=i-1; k++)
      s+=pow (dest.entry (i,k),2);
    if (s>=src.entry(i,i))
      throw invalid_argument ("Cholesky factorization is only possible for positive definite matrices: invalid argument in Martin::cholesky_factorization");    
    const double diag=sqrt(src.entry(i,i)-s);
    dest.r_entry (i,i)=diag;
    for (unsigned int j=i+1; j<=n; j++) {
      s=0;
      for (unsigned int k=1; k<=i-1; k++)
	s+=dest.entry (i,k)*dest.entry(j,k);
      dest.r_entry (j,i)=(src.entry (i,j)-s)/diag;
    }
  }
}

void Martin::cholesky_factorization (UTMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::cholesky_factorization");
  const unsigned int n=src.number_columns ();
  for (unsigned int i=1; i<=n; i++) {
    double s=0;
    for (unsigned int k=1; k<=i-1; k++)
      s+=pow (dest.entry (k,i),2);
    if (s>=src.entry(i,i))
      throw invalid_argument ("Cholesky factorization is only possible for positive definite matrices: invalid argument in Martin::cholesky_factorization");    
    const double diag=sqrt(src.entry(i,i)-s);
    dest.r_entry (i,i)=diag;
    for (unsigned int j=i+1; j<=n; j++) {
      s=0;
      for (unsigned int k=1; k<=i-1; k++)
	s+=dest.entry (k,i)*dest.entry(k,j);
      dest.r_entry (i,j)=(src.entry (i,j)-s)/diag;
    }
  }
}


// add: ------------------------------------------------------------------------------
void Martin::add (ReMatrix& dest, const Matrix& src1, const Matrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_rows ()!=src1.number_rows () || dest.number_columns ()!=src2.number_columns () || dest.number_rows ()!=src2.number_rows ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::add");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)=src1.entry (i,j)+src2.entry (i,j);
}

void Martin::add (LTMatrix& dest, const LowerTriangularMatrix& src1, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::add");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)=src1.entry (i,j)+src2.entry (i,j);
}

void Martin::add (UTMatrix& dest, const UpperTriangularMatrix& src1, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::add");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=i; j<=dest.number_columns(); j++)
      dest.r_entry (i,j)=src1.entry (i,j)+src2.entry (i,j);
}

void Martin::add (SyMatrix& dest, const SymmetricMatrix& src1, const SymmetricMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::add");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)=src1.entry (i,j)+src2.entry (i,j);
}

void Martin::add (DiMatrix& dest, const DiagonalMatrix& src1, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::add");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    dest.r_entry (i,i)=src1.entry (i,i)+src2.entry (i,i);
}


// subtract: ------------------------------------------------------------------------------
void Martin::subtract (ReMatrix& dest, const Matrix& src1, const Matrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_rows ()!=src1.number_rows () || dest.number_columns ()!=src2.number_columns () || dest.number_rows ()!=src2.number_rows ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)=src1.entry (i,j)-src2.entry (i,j);
}

void Martin::subtract (LTMatrix& dest, const LowerTriangularMatrix& src1, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)=src1.entry (i,j)-src2.entry (i,j);
}

void Martin::subtract (UTMatrix& dest, const UpperTriangularMatrix& src1, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=i; j<=dest.number_columns(); j++)
      dest.r_entry (i,j)=src1.entry (i,j)-src2.entry (i,j);
}

void Martin::subtract (SyMatrix& dest, const SymmetricMatrix& src1, const SymmetricMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)=src1.entry (i,j)-src2.entry (i,j);
}

void Martin::subtract (DiMatrix& dest, const DiagonalMatrix& src1, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (dest.number_columns ()!=src1.number_columns () || dest.number_columns ()!=src2.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    dest.r_entry (i,i)=src1.entry (i,i)-src2.entry (i,i);
}


// multiply: ------------------------------------------------------------------------------
void Martin::multiply (DiMatrix& dest, const DiagonalMatrix& src1, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    dest.r_entry (i,i)=src1.entry(i,i)*src2.entry(i,i);
  }
}


void Martin::multiply (LTMatrix& dest, const LowerTriangularMatrix& src1, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int j=1; j<=dest.number_columns (); j++) {
    const double& d (src2.entry (j,j));
    for (unsigned int i=j; i<=dest.number_rows(); i++)
      dest.r_entry (i,j)=src1.entry(i,j)*d;
  }
}

void Martin::multiply (UTMatrix& dest, const UpperTriangularMatrix& src1, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int j=1; j<=dest.number_columns (); j++) {
    const double& d (src2.entry (j,j));
    for (unsigned int i=1; i<=j; i++)
      dest.r_entry (i,j)=src1.entry(i,j)*d;
  }
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int j=1; j<=dest.number_columns (); j++) {
    const double& d (src2.entry (j,j));
    for (unsigned int i=1; i<=dest.number_rows(); i++)
      dest.r_entry (i,j)=src1.entry(i,j)*d;
  }
}

void Martin::multiply (LTMatrix& dest, const DiagonalMatrix& src1, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ())
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    const double& d (src1.entry (i,i));
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)=d*src2.entry(i,j);
  }
}

void Martin::multiply (UTMatrix& dest, const DiagonalMatrix& src1, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ())
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    const double& d (src1.entry (i,i));
    for (unsigned int j=i; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)=d*src2.entry(i,j);
  }
}

void Martin::multiply (ReMatrix& dest, const DiagonalMatrix& src1, const Matrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    const double& d (src1.entry (i,i));
    for (unsigned int j=1; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)=d*src2.entry(i,j);
  }
}

void Martin::multiply (LTMatrix& dest, const LowerTriangularMatrix& src1, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double s=0;
      for (unsigned int k=j; k<=i; k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const UpperTriangularMatrix& src1, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      double s=0;
      for (unsigned int k=max(i,j); k<=src1.number_columns(); k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      double s=0;
      for (unsigned int k=j; k<=src1.number_columns(); k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const LowerTriangularMatrix& src1, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      double s=0;
      for (unsigned int k=1; k<=min(i,j); k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const LowerTriangularMatrix& src1, const Matrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns (); j++) {
      double s=0;
      for (unsigned int k=1; k<=i; k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}


void Martin::multiply (UTMatrix& dest, const UpperTriangularMatrix& src1, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      double s=0;
      for (unsigned int k=i; k<=j; k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns (); j++) {
      double s=0;
      for (unsigned int k=1; k<=j; k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const UpperTriangularMatrix& src1, const Matrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns (); j++) {
      double s=0;
      for (unsigned int k=i; k<=src1.number_columns(); k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const Matrix& src2) throw (invalid_argument) {
  if (dest.number_rows ()!=src1.number_rows () || src1.number_columns ()!=src2.number_rows () || src2.number_columns ()!=dest.number_columns ()) 
    throw invalid_argument ("Wrong matrix dimensions in Martin::multiply");
  for (unsigned int i=1; i<=dest.number_rows (); i++) {
    for (unsigned int j=1; j<=dest.number_columns (); j++) {
      double s=0;
      for (unsigned int k=1; k<=src1.number_columns(); k++) 
	s+=src1.entry(i,k)*src2.entry(k,j);
      dest.r_entry (i,j)=s;
    }
  }
}


void Martin::multiply (LTMatrix& dest, const DiMatrix& src1, const LTMatrix& src2) throw (invalid_argument) {
  const DiagonalMatrix& s1 (src1);
  const LowerTriangularMatrix& s2 (src2);
  multiply (dest, s1, s2);
}

void Martin::multiply (UTMatrix& dest, const DiMatrix& src1, const UTMatrix& src2) throw (invalid_argument) {
  const DiagonalMatrix& s1 (src1);
  const UpperTriangularMatrix& s2 (src2);
  multiply (dest, s1, s2);
}

void Martin::multiply (ReMatrix& dest, const DiMatrix& src1, const Matrix& src2) throw (invalid_argument) {
  const DiagonalMatrix& s1 (src1);
  multiply (dest, s1, src2);
}

void Martin::multiply (LTMatrix& dest, const LTMatrix& src1, const DiMatrix& src2) throw (invalid_argument) {
  const LowerTriangularMatrix& s1 (src1);
  const DiagonalMatrix& s2 (src2);
  multiply (dest, s1, s2);
}
  
void Martin::multiply (UTMatrix& dest, const UTMatrix& src1, const DiMatrix& src2) throw (invalid_argument) {
  const UpperTriangularMatrix& s1 (src1);
  const DiagonalMatrix& s2 (src2);
  multiply (dest, s1, s2);
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const DiMatrix& src2) throw (invalid_argument) {
  const DiagonalMatrix& s2 (src2);
  multiply (dest, src1, s2);
}

void Martin::multiply (ReMatrix& dest, const LTMatrix& src1, const UTMatrix& src2) throw (invalid_argument) {
  const LowerTriangularMatrix& s1 (src1);
  const UpperTriangularMatrix& s2 (src2);
  multiply (dest, s1, s2);
}

void Martin::multiply (ReMatrix& dest, const UTMatrix& src1, const LTMatrix& src2) throw (invalid_argument) {
  const UpperTriangularMatrix& s1 (src1);
  const LowerTriangularMatrix& s2 (src2);
  multiply (dest, s1, s2);
}

void Martin::multiply (ReMatrix& dest, const LTMatrix& src1, const Matrix& src2) throw (invalid_argument) {
  const LowerTriangularMatrix& s1 (src1);
  multiply (dest, s1, src2);
}

void Martin::multiply (ReMatrix& dest, const UTMatrix& src1, const Matrix& src2) throw (invalid_argument) {
  const UpperTriangularMatrix& s1 (src1);
  multiply (dest, s1, src2);
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const LTMatrix& src2) throw (invalid_argument) {
  const LowerTriangularMatrix& s2 (src2);
  multiply (dest, src1, s2);
}

void Martin::multiply (ReMatrix& dest, const Matrix& src1, const UTMatrix& src2) throw (invalid_argument) {
  const UpperTriangularMatrix& s2 (src2);
  multiply (dest, src1, s2);
}



// transpose: ------------------------------------------------------------------------------
void Martin::transpose (ReMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (dest.number_rows ()!=src.number_columns () || src.number_rows ()!=dest.number_columns ())
    throw invalid_argument ("Wrong matrix dimensions in Martin::transpose");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)=src.entry (j,i);
}

void Martin::transpose (SyMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument) {
  copy (dest, src);
}

void Martin::transpose (LTMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (dest.number_rows ()!=src.number_columns ())
    throw invalid_argument ("Wrong matrix dimensions in Martin::transpose");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)=src.entry (j,i);
}

void Martin::transpose (UTMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (dest.number_rows ()!=src.number_columns ())
    throw invalid_argument ("Wrong matrix dimensions in Martin::transpose");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=i; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)=src.entry (j,i);
}

void Martin::transpose (DiMatrix& dest, const DiagonalMatrix& src) throw (invalid_argument) {
  copy (dest, src);
}


// inv und gauss: ------------------------------------------------------------------------------
void Martin::gauss (ReMatrix& dest, const UpperTriangularMatrix& src1, const Matrix& src2) throw (std::invalid_argument) {
  if (dest.number_rows()!=src1.number_columns() || src1.number_rows()!=src2.number_rows() || src2.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::gauss");
  unsigned int dim = src1.number_rows();
  unsigned int ldim = src2.number_columns();
  // initializing dest
  copy (dest, src2);
  // Gauss algorithm
  for (unsigned int i=dim; i>=1; i--) {
    const double diag = src1.entry (i,i);
    if (diag==0) 
      throw invalid_argument ("invalid argument in Martin::inv: irregular matrix");
    for (unsigned int j=1; j<=ldim; j++)
      dest.r_entry (i,j)/=diag;
    for (unsigned int k=i-1; k>=1; k--) {
      if (src1.entry (k,i)!=0) {
	const double fak=-src1.entry (k,i);
	for (unsigned int j=1; j<=ldim; j++) {
	  dest.r_entry(k,j)+=fak*dest.entry(i,j);
	}
      }
    }
  }
}

void Martin::inv (UTMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  const unsigned int n=src.number_columns();
  if (dest.number_columns()!=n) 
    throw invalid_argument ("invalid matrix dimension in Martin::inv");
  // initializing dest
  for (unsigned int i=1; i<=n; i++) {
    dest.r_entry (i,i)=1;
    for (unsigned int j=i+1; j<=n; j++) {
      dest.r_entry (i,j)=0;
    }
  }
  // Gauss algorithm
  for (unsigned int i=n; i>=1; i--) {
    const double diag = src.entry (i,i);
    if (diag==0) 
      throw invalid_argument ("invalid argument in Martin::inv: irregular matrix");
    for (unsigned int j=i; j<=n; j++)
      dest.r_entry (i,j)/=diag;
    for (unsigned int k=i-1; k>=1; k--) {
      if (src.entry (k,i)!=0) {
	const double fak=-src.entry (k,i);
	for (unsigned int j=i; j<=n; j++) {
	  dest.r_entry(k,j)+=fak*dest.entry(i,j);
	}
      }
    }
  }
}

void Martin::gauss (ReMatrix& dest, const LowerTriangularMatrix& src1, const Matrix& src2) throw (std::invalid_argument) {
  if (dest.number_rows()!=src1.number_columns() || src1.number_rows()!=src2.number_rows() || src2.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::gauss");
  unsigned int dim = src1.number_rows();
  unsigned int ldim = src2.number_columns();
  // initializing dest
  copy (dest, src2);
  // Gauss algorithm
  for (unsigned int i=1; i<=dim; i++) {
    const double diag = src1.entry (i,i);
    if (diag==0) 
      throw invalid_argument ("invalid argument in Martin::inv: irregular matrix");
    for (unsigned int j=1; j<=ldim; j++)
      dest.r_entry (i,j)/=diag;
    for (unsigned int k=i+1; k<=dim; k++) {
      if (src1.entry (k,i)!=0) {
	const double fak=-src1.entry (k,i);
	for (unsigned int j=1; j<=ldim; j++) {
	  dest.r_entry(k,j)+=fak*dest.entry(i,j);
	}
      }
    }
  }
}
  

void Martin::inv (LTMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  const unsigned int n=src.number_columns();
  if (dest.number_columns()!=n) 
    throw invalid_argument ("invalid matrix dimension in Martin::inv");
  // initializing dest
  for (unsigned int i=1; i<=n; i++) {
    dest.r_entry (i,i)=1;
    for (unsigned int j=1; j<i; j++) {
      dest.r_entry (i,j)=0;
    }
  }
  // Gauss algorithm
  for (unsigned int i=1; i<=n; i++) {
    const double diag = src.entry (i,i);
    if (diag==0) 
      throw invalid_argument ("invalid argument in Martin::inv: irregular matrix");
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)/=diag;
    for (unsigned int k=i+1; k<=n; k++) {
      if (src.entry (k,i)!=0) {
	const double fak=-src.entry (k,i);
	for (unsigned int j=1; j<=i; j++) {
	  dest.r_entry(k,j)+=fak*dest.entry(i,j);
	}
      }
    }
  }
}

void Martin::inv (SyMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument, bad_alloc) {
  if (src.number_columns()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::inv");
  UTMatrix cholesky (src.number_rows());
  UTMatrix inv_cholesky (src.number_rows());
  cholesky_factorization (cholesky, src);
  inv (inv_cholesky, cholesky);
  // return inv_cholesky*inv_cholesky^T
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=max(i,j); k<=dest.number_rows(); k++)
	sum+=inv_cholesky.entry(i,k)*inv_cholesky.entry(j,k);
      dest.r_entry(i,j)=sum;
    }
  }
}

void Martin::gauss (ReMatrix& dest, const DiagonalMatrix& src1, const Matrix& src2) throw (std::invalid_argument) {
  if (dest.number_rows()!=src1.number_columns() || src1.number_rows()!=src2.number_rows() || src2.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::gauss");
  unsigned int dim = src1.number_rows();
  unsigned int ldim = src2.number_columns();
  for (unsigned int i=1; i<=dim; i++) {
    double f = src1.entry(i,i);
    for (unsigned int j=1; j<=ldim; j++)
      dest.r_entry(i,j)=src2.entry(i,j)/f;
  }
}

void Martin::gauss (ReMatrix& dest, const Matrix& src1, const Matrix& src2) throw (invalid_argument, bad_alloc) {
  // Gauss algorithm
  if (src1.number_rows()!=src1.number_columns() || dest.number_rows()!=src1.number_columns() || src1.number_rows()!=src2.number_rows() || src2.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::gauss");
  unsigned int dim = src1.number_rows();
  unsigned int ldim = src2.number_columns();
  ReMatrix s (dim,dim);
  // init
  copy (s, src1);
  copy (dest, src2);
  
  // iterations
  for (unsigned int k=1; k<=dim; k++) {
    if (s.entry(k,k)==0) {
      // search entry!=0 in the k-th column
      unsigned int n=k+1;
      while (n<=dim && s.entry(n,k)==0)
	n++;
      if (n>dim) // matrix is not invertible
	throw invalid_argument ("irregular matrix in Martin::gauss");
      else { // permute k-th and n-th row
	for (unsigned int j=1; j<=ldim; j++) {
	  double sw=dest.entry(k,j);
	  dest.r_entry(k,j)=dest.entry(n,j);
	  dest.r_entry(n,j)=sw;
	}
	for (unsigned int j=k; j<=dim; j++) {
	  double sw=s.entry(k,j);
	  s.r_entry(k,j)=s.entry(n,j);
	  s.r_entry(n,j)=sw;
	}
      }
    }
    // normalize k-th row
    const double pivot=s.entry(k,k);
    for (unsigned int j=k+1; j<=dim; j++)
      s.r_entry(k,j)/=pivot;
    for (unsigned int j=1; j<=ldim; j++)
      dest.r_entry(k,j)/=pivot;
    // add k-th row to the other rows to obtain zeros in the k-th column
    for (unsigned int i=1; i<=dim; i++) 
      if (i!=k) {
	const double multiplier=-s.entry (i,k);
	for (unsigned int j=k+1; j<=dim; j++)
	  s.r_entry(i,j)+=multiplier*s.entry(k,j);
	for (unsigned int j=1; j<=ldim; j++)
	  dest.r_entry(i,j)+=multiplier*dest.entry(k,j);
      }
  }
}

void Martin::inv (ReMatrix& dest, const Matrix& src) throw (invalid_argument, bad_alloc) {
  DiMatrix identity (src.number_rows());
  for (unsigned int i=1; i<=src.number_rows(); i++)
    identity.r_entry(i,i)=1;
  gauss (dest, src, identity);
}

void Martin::inv (DiMatrix& dest, const DiagonalMatrix& src) throw (invalid_argument) {
  // Gauss algorithm
  if (dest.number_columns()!=src.number_columns()) 
    throw invalid_argument ("invalid matrix dimension in Martin::inv");
  for (unsigned int i=1; i<=dest.number_rows(); i++)
    dest.r_entry (i,i)=1.0/src.entry (i,i);
}

void Martin::gauss (ReMatrix& dest, const LTMatrix& src1, const Matrix& src2) throw (std::invalid_argument) {
  const LowerTriangularMatrix& src11 (src1);
  gauss (dest, src11, src2);
}

void Martin::gauss (ReMatrix& dest, const UTMatrix& src1, const Matrix& src2) throw (std::invalid_argument) {
  const UpperTriangularMatrix& src11 (src1);
  gauss (dest, src11, src2);
}

void Martin::gauss (ReMatrix& dest, const DiMatrix& src1, const Matrix& src2) throw (std::invalid_argument) {
  const DiagonalMatrix& src11 (src1);
  gauss (dest, src11, src2);
}


// determinant: ------------------------------------------------------------------------------
double Martin::determinant (const UpperTriangularMatrix& src) throw () {
  double prod=1;
  for (unsigned int i=1; i<=src.number_columns(); i++)
    prod*=src.entry (i,i);
  return prod;
}

double Martin::determinant (const LowerTriangularMatrix& src) throw () {
  double prod=1;
  for (unsigned int i=1; i<=src.number_columns(); i++)
    prod*=src.entry (i,i);
  return prod;
}

double Martin::determinant (const Matrix& src) throw (invalid_argument, bad_alloc) {
  if (src.number_rows()!=src.number_columns())
      throw invalid_argument ("determinant is only defined for square matrices: invalid argument in Martin::determinant");
  if (src.number_rows()==1)
    return src.entry(1,1);
  else {
    double res=0;
    double vz=1;
    ReMatrix s (src.number_rows()-1, src.number_rows ()-1);
    for (unsigned int i=1; i<=s.number_rows(); i++) {
      for (unsigned int j=1; j<=s.number_columns(); j++) {
	s.r_entry (i,j)=src.entry (i+1, j+1);
      }
    }
    for (unsigned int i=1; i<=src.number_rows(); i++) {
      if (i>1) {
	for (unsigned int j=1; j<=s.number_columns(); j++) {
	  s.r_entry (i-1,j)=src.entry (i-1, j+1);
	}
      }
      if (src.entry (i,1)!=0)
	res+=vz*src.entry (i,1)*determinant (s);
      vz*=-1;
    }
    return res;
  }
}

bool Martin::is_positive_definite (const SymmetricMatrix& src) throw(bad_alloc) {
  // try to find a cholesky factorization
  // if successfull, src is postive definite
  LTMatrix dest (src.number_rows());
  const unsigned int n=src.number_columns ();
  for (unsigned int i=1; i<=n; i++) {
    double s=0;
    for (unsigned int k=1; k<=i-1; k++)
      s+=pow (dest.entry (i,k),2);
    if (s>src.entry(i,i))
      return false;
    const double diag=sqrt(src.entry(i,i)-s);
    dest.r_entry (i,i)=diag;
    for (unsigned int j=i+1; j<=n; j++) {
      s=0;
      for (unsigned int k=1; k<=i-1; k++)
	s+=dest.entry (i,k)*dest.entry(j,k);
      dest.r_entry (j,i)=(src.entry (i,j)-s)/diag;
    }
  }
  return true;
}

double Martin::determinant (const LTMatrix& src) throw () {
  const Martin::LowerTriangularMatrix& src2 (src);
  return Martin::determinant (src2);
}

double Martin::determinant (const UTMatrix& src) throw () {
  const Martin::UpperTriangularMatrix& src2 (src);
  return Martin::determinant (src2);
}

double Martin::determinant (const DiMatrix& src) throw () {
  const Martin::LowerTriangularMatrix& src2 (src);
  return Martin::determinant (src2);
}


// multiply_with_scalars: ------------------------------------------------------------------------------
void Martin::multiply_with_scalars (SyMatrix& dest, double s, const SymmetricMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=src.entry(i,j)*s;
    }
  }
}

void Martin::multiply_with_scalars (DiMatrix& dest, double s, const DiagonalMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)=src.entry(i,i)*s;
  }
}

void Martin::multiply_with_scalars (LTMatrix& dest, double s, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=src.entry(i,j)*s;
    }
  }
}

void Martin::multiply_with_scalars (UTMatrix& dest, double s, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=src.entry(i,j)*s;
    }
  }
}

void Martin::multiply_with_scalars (ReMatrix& dest, double s, const Matrix& src) throw (invalid_argument) {
  if ((src.number_rows()!=dest.number_rows()) || (src.number_columns()!=dest.number_columns()))
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=src.entry(i,j)*s;
    }
  }
}


// dot_product: ------------------------------------------------------------------------------
double Martin::dot_product (const ColumnVector& src1, const ColumnVector& src2) throw (invalid_argument) {
  if (src1.number_rows()!=src2.number_rows())
    throw invalid_argument ("mismatch in vector dimensions in Martin::dot_product");
  double res=0;
  for (unsigned int i=1; i<=src1.number_rows(); i++)
    res+=src1.entry(i)*src2.entry(i);
  return res;
}

double Martin::dot_product (const RowVector& src1, const RowVector& src2) throw (invalid_argument) {
  if (src1.number_columns()!=src2.number_columns())
    throw invalid_argument ("mismatch in vector dimensions in Martin::dot_product");
  double res=0;
  for (unsigned int i=1; i<=src1.number_columns(); i++)
    res+=src1.entry(i)*src2.entry(i);
  return res;
}


// trace: ------------------------------------------------------------------------------
double Martin::trace (const Matrix& src) throw (invalid_argument) {
  if (src.number_rows()!=src.number_columns())
    throw invalid_argument ("trace only defined for square matrices: invalid argument in Martin::inv");
  double sum=0;
  for (unsigned int i=1; i<=src.number_rows(); i++)
    sum+=src.entry (i,i);
  return sum;
}





