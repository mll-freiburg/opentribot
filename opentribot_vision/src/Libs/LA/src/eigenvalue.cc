
#include "eigenvalue.h"
#include <cmath>

using namespace Martin;
using namespace std;

namespace {

  void triDiag(ReMatrix& a, ColumnVector& d, ColumnVector& e) {
    // without computation of eigenvectors
    unsigned int i, j, k, l;
    double scale, hh, h, g, f;

    const unsigned int dim = a.number_rows();
  
    for (i=dim; i>=2; i--) {
      l=i-1;
      h=0;
      scale=0;
      if (l > 1) {
	for (k=1;k<=l;k++)
	  scale += abs(a.entry(i,k));
	if (scale == 0)
	  e.r_entry(i)=a.entry(i,l);
	else {
	  for (k=1;k<=l;k++) {
	    a.r_entry(i,k) /= scale;
	    h += a.entry(i,k)*a.entry(i,k);
	  }
	  f=a.entry(i,l);
	  g=(f >= 0 ? -sqrt(h) : sqrt(h));
	  e.r_entry(i)=scale*g;
	  h -= f*g;
	  a.r_entry(i,l)=f-g;
	  f=0;
	  for (j=1;j<=l;j++) {
	    g=0;
	    for (k=1;k<=j;k++)
	      g += a.entry(j,k)*a.entry(i,k);
	    for (k=j+1;k<=l;k++)
	      g += a.entry(k,j)*a.entry(i,k);
	    e.r_entry (j)=g/h;
	    f += e.entry(j)*a.r_entry(i,j);
	  }
	  hh=f/(h+h);
	  for (j=1;j<=l;j++) {
	    f=a.entry(i,j);
	    g=e.entry (j)-hh*f;
	    e.r_entry (j)=g;
	    for (k=1;k<=j;k++)
	      a.r_entry (j,k) -= (f*e.entry (k)+g*a.entry(i,k));
	  }
	}
      } else {
	e.r_entry (i)=a.entry (i,l);
      }
      d.r_entry(i)=h;
    }
    e.r_entry (1)=0.0;
    for (i=1;i<=dim;i++) {
      d.r_entry (i)=a.entry(i,i);
    }
  }

  void tQLi(ColumnVector& d, ColumnVector& e) {
    // without computation of eigenvectors
    unsigned int m,l,iter,i;
    double s, r, p, g, f, dd, c, b;
    
    const unsigned int dim = d.number_rows();

    for (i=2;i<=d.number_rows();i++) 
      e.r_entry(i-1)=e.entry(i);
    e.r_entry (d.number_rows())=0;
    for (l=1;l<=d.number_rows();l++) {
      iter=0;
      do {
	for (m=l;m+1<=dim;m++) {
	  dd=abs(d.entry(m))+abs(d.entry(m+1));
	  if (abs(e.entry(m))+dd == dd) break;
	}
	if (m != l) {
	  if (iter++ >= 30) 
	    throw invalid_argument ("Too many iterations in tqli");
	  g=(d.entry (l+1)-d.entry (l))/(2.0*e.entry (l));
	  r=sqrt(g*g+1);
	  g=d.entry(m)-d.entry(l)+e.entry(l)/(g+(g>0 ? abs(r) : -abs(r)));
	  c=1;
	  s=1;
	  p=0;
	  for (i=m-1;i>=l;i--) {
	    f=s*e.entry (i);
	    b=c*e.entry (i);
	    r=sqrt(f*f+g*g);
	    e.r_entry (i+1)=r;
	    if (r == 0) {
	      d.r_entry(i+1) -= p;
	      e.r_entry(m)=0;
	      break;
	    }
	    s=f/r;
	    c=g/r;
	    g=d.entry(i+1)-p;
	    r=(d.entry(i)-g)*s+2.0*c*b;
	    p=s*r;
	    d.r_entry(i+1)=g+p;
	    g=c*r-b;
	  }
	  if (r == 0.0 && i >= l) continue;
	  d.r_entry(l) -= p;
	  e.r_entry(l)=g;
	  e.r_entry(m)=0;
	}
      } while (m != l);
    }      
  }

  void triDiag2(ReMatrix& a, ColumnVector& d, ColumnVector& e) {
    // with computation of eigenvectors
    unsigned int i, j, k, l;
    double scale, hh, h, g, f;
  
    const unsigned int dim = a.number_rows();

    for (i=dim; i>=2; i--) {
      l=i-1;
      h=0;
      scale=0;
      if (l > 1) {
	for (k=1;k<=l;k++)
	  scale += abs(a.entry(i,k));
	if (scale == 0)
	  e.r_entry(i)=a.entry(i,l);
	else {
	  for (k=1;k<=l;k++) {
	    a.r_entry(i,k) /= scale;
	    h += a.entry(i,k)*a.entry(i,k);
	  }
	  f=a.entry(i,l);
	  g=(f >= 0 ? -sqrt(h) : sqrt(h));
	  e.r_entry(i)=scale*g;
	  h -= f*g;
	  a.r_entry(i,l)=f-g;
	  f=0;
	  for (j=1;j<=l;j++) {
	    a.r_entry (j,i)=a.entry(i,j)/h;
	    g=0;
	    for (k=1;k<=j;k++)
	      g += a.entry(j,k)*a.entry(i,k);
	    for (k=j+1;k<=l;k++)
	      g += a.entry(k,j)*a.entry(i,k);
	    e.r_entry (j)=g/h;
	    f += e.entry(j)*a.r_entry(i,j);
	  }
	  hh=f/(h+h);
	  for (j=1;j<=l;j++) {
	    f=a.entry(i,j);
	    g=e.entry (j)-hh*f;
	    e.r_entry (j)=g;
	    for (k=1;k<=j;k++)
	      a.r_entry (j,k) -= (f*e.entry (k)+g*a.entry(i,k));
	  }
	}
      } else {
	e.r_entry (i)=a.entry (i,l);
      }
      d.r_entry(i)=h;
    }
    d.r_entry (1)=0.0;
    e.r_entry (1)=0.0;
    for (i=1;i<=dim;i++) {
      l=i-1;
      if (d.entry(i)) {
	for (j=1;j<=l;j++) {
	  g=0.0;
	  for (k=1;k<=l;k++)
	    g += a.entry (i,k)*a.entry (k,j);
	  for (k=1;k<=l;k++)
	    a.r_entry(k,j) -= g*a.entry(k,i);
	}
      }
      d.r_entry (i)=a.entry(i,i);
      a.r_entry(i,i)=1.0;
      for (j=1;j<=l;j++) {
	a.r_entry(j,i)=0.0;
	a.r_entry(i,j)=0.0;
      }
    }
  }

  void tQLi2(ColumnVector& d, ColumnVector& e, ReMatrix& z) {
    // with computation of eigenvectors
    unsigned int m,l,iter,i,k;
    double s, r, p, g, f, dd, c, b;

    const unsigned int dim = d.number_rows();
    
    for (i=2;i<=dim;i++) 
      e.r_entry(i-1)=e.entry(i);
    e.r_entry (dim)=0;
    for (l=1;l<=dim;l++) {
      iter=0;
      do {
	for (m=l;m+1<=dim;m++) {
	  dd=abs(d.entry(m))+abs(d.entry(m+1));
	  if (abs(e.entry(m))+dd == dd) break;
	}
	if (m != l) {
	  if (iter++ >= 30) 
	    throw invalid_argument ("Too many iterations in tqli");
	  g=(d.entry (l+1)-d.entry (l))/(2.0*e.entry (l));
	  r=sqrt(g*g+1);
	  g=d.entry(m)-d.entry(l)+e.entry(l)/(g+(g>0 ? abs(r) : -abs(r)));
	  c=1;
	  s=1;
	  p=0;
	  for (i=m-1;i>=l;i--) {
	    f=s*e.entry (i);
	    b=c*e.entry (i);
	    r=sqrt(f*f+g*g);
	    e.r_entry (i+1)=r;
	    if (r == 0) {
	      d.r_entry(i+1) -= p;
	      e.r_entry(m)=0;
	      break;
	    }
	    s=f/r;
	    c=g/r;
	    g=d.entry(i+1)-p;
	    r=(d.entry(i)-g)*s+2.0*c*b;
	    p=s*r;
	    d.r_entry(i+1)=g+p;
	    g=c*r-b;
	    for (k=1;k<=dim;k++) {
	      f=z.entry(k,i+1);
	      z.r_entry(k,i+1)=s*z.entry(k,i)+c*f;
	      z.r_entry(k,i)=c*z.entry(k,i)-s*f;
	    }
	  }
	  if (r == 0.0 && i >= l) continue;
	  d.r_entry(l) -= p;
	  e.r_entry(l)=g;
	  e.r_entry(m)=0;
	}
      } while (m != l);
    }      
  }

}


void Martin::eigenvalue (ColumnVector& dest, const SymmetricMatrix& src) throw (bad_alloc, invalid_argument) {
  // use the method of housholder transformation, 
  // see Press et al., Numerical recipes in C
  if (dest.number_rows()!=src.number_rows())
    throw invalid_argument ("mismatch in matrix dimension in Martin::eigenvalue");

  const unsigned int num_row = src.number_rows();
  ReMatrix a (num_row, num_row);
  for (unsigned int i=1; i<=num_row; i++)
    for (unsigned int j=1; j<=i; j++)
      a.r_entry (i,j)=a.r_entry(j,i)=src.entry(i,j);
  ColumnVector e (num_row);
  triDiag (a, dest, e);
  tQLi (dest, e);
}

void Martin::eigenvalue (ColumnVector& dest, ReMatrix& a, const SymmetricMatrix& src) throw (bad_alloc, invalid_argument) {
  // use the method of housholder transformation, 
  // see Press et al., Numerical recipes in C
  if (dest.number_rows()!=src.number_rows() || dest.number_rows()!=a.number_rows() || a.number_rows()!=a.number_columns())
    throw invalid_argument ("mismatch in matrix dimension in Martin::eigenvalue");
  
  const unsigned int num_row = src.number_rows();
  for (unsigned int i=1; i<=num_row; i++)
    for (unsigned int j=1; j<=i; j++)
      a.r_entry (i,j)=a.r_entry(j,i)=src.entry(i,j);
  ColumnVector e (num_row);
  triDiag2 (a, dest, e);
  tQLi2 (dest, e, a);
}


