
#include "matrix_compound_operations.h"
#include <algorithm>

using namespace Martin;
using namespace std;

void Martin::identity (DiMatrix& dest, double d) throw () {
  unsigned int n=dest.number_rows();
  for (unsigned int i=1; i<=n; i++)
    dest.r_entry(i,i)=d;
}

// multiply_with_transpose/multiply_transpose_with: -----------------------------------------------------------------
void Martin::multiply_with_transpose (SyMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_transpose");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=max(i,j); k<=dest.number_rows(); k++)
	sum+=src.entry(i,k)*src.entry(j,k);
      dest.r_entry(i,j)=sum;
    }
  }
}

void Martin::multiply_with_transpose (SyMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_transpose");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=min(i,j); k++)
	sum+=src.entry(i,k)*src.entry(j,k);
      dest.r_entry(i,j)=sum;
    }
  }
}

void Martin::multiply_transpose_with (SyMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_rows()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_transpose_with");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=min(i,j); k++)
	sum+=src.entry(k,i)*src.entry(k,j);
      dest.r_entry(i,j)=sum;
    }
  } 
}

void Martin::multiply_transpose_with (SyMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_rows()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_transpose_with");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=max(i,j); k<=src.number_rows(); k++)
	sum+=src.entry(k,i)*src.entry(k,j);
      dest.r_entry(i,j)=sum;
    }
  } 
}

void Martin::multiply_with_transpose (SyMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_rows()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_transpose");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=src.number_columns(); k++)
	sum+=src.entry(i,k)*src.entry(j,k);
      dest.r_entry(i,j)=sum;
    }
  }
}

void Martin::multiply_transpose_with (SyMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_rows()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_transpose_with");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=src.number_rows(); k++)
	sum+=src.entry(k,i)*src.entry(k,j);
      dest.r_entry(i,j)=sum;
    }
  } 
}

void Martin::multiply_with_transpose (SyMatrix& dest, const LTMatrix& src) throw (invalid_argument) {
  const LowerTriangularMatrix& s (src);
  multiply_with_transpose (dest, s);
}

void Martin::multiply_with_transpose (SyMatrix& dest, const UTMatrix& src) throw (invalid_argument) {
  const UpperTriangularMatrix& s (src);
  multiply_with_transpose (dest, s);
}


void Martin::multiply_transpose_with (SyMatrix& dest, const LTMatrix& src) throw (invalid_argument) {
  const LowerTriangularMatrix& s (src);
  multiply_transpose_with (dest, s);
}

void Martin::multiply_transpose_with (SyMatrix& dest, const UTMatrix& src) throw (invalid_argument) {
  const UpperTriangularMatrix& s (src);
  multiply_transpose_with (dest, s);
}

// add_with_scalars: -----------------------------------------------------------------
void Martin::add_with_scalars (ReMatrix& dest, double s1, const Matrix& src1, double s2, const Matrix& src2) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src1.number_columns()!=dest.number_columns() || src2.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j);
    }
  }
}
  
void Martin::add_with_scalars (LTMatrix& dest, double s1, const LowerTriangularMatrix& src1, double s2, const LowerTriangularMatrix& src2) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (UTMatrix& dest, double s1, const UpperTriangularMatrix& src1, double s2, const UpperTriangularMatrix& src2) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (DiMatrix& dest, double s1, const DiagonalMatrix& src1, double s2, const DiagonalMatrix& src2) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)=s1*src1.entry(i,i)+s2*src2.entry(i,i);
  }
}
  
void Martin::add_with_scalars (SyMatrix& dest, double s1, const SymmetricMatrix& src1, double s2, const SymmetricMatrix& src2) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j);
    }
  }
}



void Martin::add_with_scalars (ReMatrix& dest, double s1, const Matrix& src1, double s2, const Matrix& src2, double s3, const Matrix& src3) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src1.number_columns()!=dest.number_columns() || src2.number_columns()!=dest.number_columns() || src3.number_rows()!=dest.number_rows() || src3.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (SyMatrix& dest, double s1, const SymmetricMatrix& src1, double s2, const SymmetricMatrix& src2, double s3, const SymmetricMatrix& src3) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (LTMatrix& dest, double s1, const LowerTriangularMatrix& src1, double s2, const LowerTriangularMatrix& src2, double s3, const LowerTriangularMatrix& src3) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (UTMatrix& dest, double s1, const UpperTriangularMatrix& src1, double s2, const UpperTriangularMatrix& src2, double s3, const UpperTriangularMatrix& src3) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (DiMatrix& dest, double s1, const DiagonalMatrix& src1, double s2, const DiagonalMatrix& src2, double s3, const DiagonalMatrix& src3) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)=s1*src1.entry(i,i)+s2*src2.entry(i,i)+s3*src3.entry(i,i);
  }
}



void Martin::add_with_scalars (ReMatrix& dest, double s1, const Matrix& src1, double s2, const Matrix& src2, double s3, const Matrix& src3, double s4, const Matrix& src4) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src1.number_columns()!=dest.number_columns() || src2.number_columns()!=dest.number_columns() || src3.number_rows()!=dest.number_rows() || src4.number_rows()!=dest.number_rows() || src3.number_columns()!=dest.number_columns() || src4.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j)+s4*src4.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (SyMatrix& dest, double s1, const SymmetricMatrix& src1, double s2, const SymmetricMatrix& src2, double s3, const SymmetricMatrix& src3, double s4, const SymmetricMatrix& src4) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows() || src4.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j)+s4*src4.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (LTMatrix& dest, double s1, const LowerTriangularMatrix& src1, double s2, const LowerTriangularMatrix& src2, double s3, const LowerTriangularMatrix& src3, double s4, const LowerTriangularMatrix& src4) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows() || src4.number_rows()!=dest.number_rows() )
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j)+s4*src4.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (UTMatrix& dest, double s1, const UpperTriangularMatrix& src1, double s2, const UpperTriangularMatrix& src2, double s3, const UpperTriangularMatrix& src3, double s4, const UpperTriangularMatrix& src4) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows() || src4.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*src1.entry(i,j)+s2*src2.entry(i,j)+s3*src3.entry(i,j)+s4*src4.entry(i,j);
    }
  }
}

void Martin::add_with_scalars (DiMatrix& dest, double s1, const DiagonalMatrix& src1, double s2, const DiagonalMatrix& src2, double s3, const DiagonalMatrix& src3, double s4, const DiagonalMatrix& src4) throw (invalid_argument) {
  if (src1.number_rows()!=dest.number_rows() || src2.number_rows()!=dest.number_rows() || src3.number_rows()!=dest.number_rows() || src4.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)=s1*src1.entry(i,i)+s2*src2.entry(i,i)+s3*src3.entry(i,i)+s4*src4.entry(i,i);
  }
}


// multiply_with_transpose_with_matrix/multiply_with_matrix_with_transpose: ------------------------------------------------
void Martin::multiply_with_transpose_with_matrix (SyMatrix& dest, const Matrix& src1, const SymmetricMatrix& src2) throw (invalid_argument) {
  const unsigned int p=src1.number_columns();
  const unsigned int q=src1.number_rows();
  if (dest.number_rows() != p || q != src2.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_transpose_with_matrix");
  ReMatrix zm (q, p);
  for (unsigned int i=1; i<=q; i++)
    for (unsigned int j=1; j<=p; j++) {
      double sum=0;
      for (unsigned int k=1; k<=q; k++)
	sum+=src2.entry (i,k)*src1.entry(k,j);
      zm.r_entry (i,j)=sum;
    }
  for (unsigned int i=1; i<=p; i++)
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=q; k++)
	sum+=src1.entry (k,i)*zm.entry(k,j);
      dest.r_entry (i,j)=sum;
    }
}

void Martin::multiply_with_matrix_with_transpose (SyMatrix& dest, const Matrix& src1, const SymmetricMatrix& src2) throw (invalid_argument) {
  const unsigned int p=src1.number_columns();
  const unsigned int q=src1.number_rows();
  if (dest.number_rows() != q || p != src2.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::multiply_with_transpose_with_matrix");
  ReMatrix zm (p, q);
  for (unsigned int i=1; i<=p; i++)
    for (unsigned int j=1; j<=q; j++) {
      double sum=0;
      for (unsigned int k=1; k<=p; k++)
	sum+=src2.entry (i,k)*src1.entry(j,k);
      zm.r_entry (i,j)=sum;
    }
  for (unsigned int i=1; i<=q; i++)
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=p; k++)
	sum+=src1.entry (i,k)*zm.entry(k,j);
      dest.r_entry (i,j)=sum;
    }
}


// add_to: ----------------------------------------------------------------------------------------------------------
void Martin::add_to (ReMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns() || src.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)+=src.entry(i,j);
    }
  }
}  

void Martin::add_to (LTMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)+=src.entry(i,j);
    }
  }
}  

void Martin::add_to (UTMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)+=src.entry(i,j);
    }
  }
}  

void Martin::add_to (SyMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)+=src.entry(i,j);
    }
  }
}  

void Martin::add_to (DiMatrix& dest, const DiagonalMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)+=src.entry(i,i);
  }
}  


// subtract_from: --------------------------------------------------------------------------------
void Martin::subtract_from (ReMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns () || dest.number_rows ()!=src.number_rows ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract_from");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=dest.number_columns (); j++)
      dest.r_entry (i,j)-=src.entry (i,j);
}

void Martin::subtract_from (SyMatrix& dest, const SymmetricMatrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract_from");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)-=src.entry (i,j);
}

void Martin::subtract_from (LTMatrix& dest, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract_from");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=1; j<=i; j++)
      dest.r_entry (i,j)-=src.entry (i,j);
}

void Martin::subtract_from (UTMatrix& dest, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract_from");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    for (unsigned int j=i; j<=dest.number_columns(); j++)
      dest.r_entry (i,j)-=src.entry (i,j);
}

void Martin::subtract_from (DiMatrix& dest, const DiagonalMatrix& src) throw (invalid_argument) {
  if (dest.number_columns ()!=src.number_columns ()) 
    throw invalid_argument ("Differing matrix dimensions in Martin::subtract_from");
  for (unsigned int i=1; i<=dest.number_rows (); i++)
    dest.r_entry (i,i)-=src.entry (i,i);
}


// add_to_with_scalars: -----------------------------------------------------------------------------------
void Martin::add_to_with_scalars (ReMatrix& dest, double s1, double s2, const Matrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns() || src.number_rows()!=dest.number_rows())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*dest.entry(i,j)+s2*src.entry(i,j);
    }
  }
}

void Martin::add_to_with_scalars (SyMatrix& dest, double s1, double s2, const SymmetricMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*dest.entry(i,j)+s2*src.entry(i,j);
    }
  }
}

void Martin::add_to_with_scalars (LTMatrix& dest, double s1, double s2, const LowerTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)=s1*dest.entry(i,j)+s2*src.entry(i,j);
    }
  }
}

void Martin::add_to_with_scalars (UTMatrix& dest, double s1, double s2, const UpperTriangularMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)=s1*dest.entry(i,j)+s2*src.entry(i,j);
    }
  }
}

void Martin::add_to_with_scalars (DiMatrix& dest, double s1, double s2, const DiagonalMatrix& src) throw (invalid_argument) {
  if (src.number_columns()!=dest.number_columns())
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_with_scalars");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)=s1*dest.entry(i,i)+s2*src.entry(i,i);
  }
}


// multiply_with_scalars: -----------------------------------------------------------------------------
void Martin::multiply_with_scalars (ReMatrix& dest, double s) throw () {
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)*=s;
    }
  }
}

void Martin::multiply_with_scalars (SyMatrix& dest, double s) throw () {
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)*=s;
    }
  }
}

void Martin::multiply_with_scalars (LTMatrix& dest, double s) throw () {
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      dest.r_entry(i,j)*=s;
    }
  }
}

void Martin::multiply_with_scalars (UTMatrix& dest, double s) throw () {
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=i; j<=dest.number_columns(); j++) {
      dest.r_entry(i,j)*=s;
    }
  }
}

void Martin::multiply_with_scalars (DiMatrix& dest, double s) throw () {
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    dest.r_entry(i,i)*=s;
  }
}


// add_to_multiply_with_transpose: ---------------------------------------------------------------------------
void Martin::add_to_multiply_with_transpose (SyMatrix& dest, const Matrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_rows()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_multiply_with_transpose");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=src.number_columns(); k++) {
	sum+=src.entry(i,k)*src.entry(j,k);
      }
      dest.r_entry(i,j)+=sum;
    }
  }
}


// add_to_with_scalars_multiply_with_transpose: ---------------------------------------------------------------
void Martin::add_to_with_scalars_multiply_with_transpose (SyMatrix& dest, double s1, double s2, const Matrix& src) throw (invalid_argument) {
  if (src.number_rows()!=dest.number_rows()) 
    throw invalid_argument ("mismatch in matrix dimensions in Martin::add_to_with_scalars_multiply_with_transpose");
  for (unsigned int i=1; i<=dest.number_rows(); i++) {
    for (unsigned int j=1; j<=i; j++) {
      double sum=0;
      for (unsigned int k=1; k<=src.number_columns(); k++) {
	sum+=src.entry(i,k)*src.entry(j,k);
      }
      dest.r_entry(i,j)=s1*dest.entry(i,j)+s2*sum;
    }
  }
}


