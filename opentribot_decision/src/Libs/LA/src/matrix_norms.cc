
#include "matrix_norms.h"
#include <cmath>

using namespace Martin;
using namespace std;

namespace {
  inline void maxim (double& a1, double a2) throw () {
    if (a2>a1) a1=a2;
  }
}

double Martin::euklid_norm (const ColumnVector& src) throw () { 
  double sum=0;
  for (unsigned int i=1; i<=src.number_rows(); i++)
    sum+=pow(src.entry(i),2);
  return sqrt(sum);
}

double Martin::euklid_distance (const ColumnVector& src1, const ColumnVector& src2) throw (invalid_argument) { 
  if (src1.number_rows()!=src2.number_rows())
    throw invalid_argument("mismatch in vector dimension in Martin::euklid_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_rows(); i++)
    sum+=pow(src1.entry(i)-src2.entry(i),2);
  return sqrt(sum);
}

double Martin::euklid_norm (const RowVector& src) throw () { 
  double sum=0;
  for (unsigned int i=1; i<=src.number_columns(); i++)
    sum+=pow(src.entry(i),2);
  return sqrt(sum);
}

double Martin::euklid_distance (const RowVector& src1, const RowVector& src2) throw (invalid_argument) { 
  if (src1.number_columns()!=src2.number_columns())
    throw invalid_argument("mismatch in vector dimension in Martin::euklid_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_columns(); i++)
    sum+=pow(src1.entry(i)-src2.entry(i),2);
  return sqrt(sum);
}



double Martin::manhattan_norm (const ColumnVector& src) throw () { 
  double sum=0;
  for (unsigned int i=1; i<=src.number_rows(); i++)
    sum+=abs(src.entry(i));
  return sum;
}

double Martin::manhattan_distance (const ColumnVector& src1, const ColumnVector& src2) throw (invalid_argument) { 
  if (src1.number_rows()!=src2.number_rows())
    throw invalid_argument("mismatch in vector dimension in Martin::manhattan_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_rows(); i++)
    sum+=abs(src1.entry(i)-src2.entry(i));
  return sum;
}

double Martin::manhattan_norm (const RowVector& src) throw () { 
  double sum=0;
  for (unsigned int i=1; i<=src.number_columns(); i++)
    sum+=abs(src.entry(i));
  return sum;
}

double Martin::manhattan_distance (const RowVector& src1, const RowVector& src2) throw (invalid_argument) { 
  if (src1.number_columns()!=src2.number_columns())
    throw invalid_argument("mismatch in vector dimension in Martin::manhattan_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_columns(); i++)
    sum+=abs(src1.entry(i)-src2.entry(i));
  return sum;
}



double Martin::maximum_norm (const ColumnVector& src) throw () { 
  double sum=0;
  for (unsigned int i=1; i<=src.number_rows(); i++)
    maxim (sum, abs(src.entry(i)));
  return sum;
}

double Martin::maximum_distance (const ColumnVector& src1, const ColumnVector& src2) throw (invalid_argument) { 
  if (src1.number_rows()!=src2.number_rows())
    throw invalid_argument("mismatch in vector dimension in Martin::maximum_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_rows(); i++)
    maxim (sum, abs(src1.entry(i)-src2.entry(i)));
  return sum;
}

double Martin::maximum_norm (const RowVector& src) throw () { 
  double sum=0;
  for (unsigned int i=1; i<=src.number_columns(); i++)
    maxim (sum, abs(src.entry(i)));
  return sum;
}

double Martin::maximum_distance (const RowVector& src1, const RowVector& src2) throw (invalid_argument) { 
  if (src1.number_columns()!=src2.number_columns())
    throw invalid_argument("mismatch in vector dimension in Martin::maximum_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_columns(); i++)
    maxim (sum, abs(src1.entry(i)-src2.entry(i)));
  return sum;
}



double Martin::minkowsky_norm (const ColumnVector& src, double degree) throw (invalid_argument) { 
  if (degree<=0)
    throw invalid_argument ("invalid parameter in Martin::minkowsky_norm");
  double sum=0;
  for (unsigned int i=1; i<=src.number_rows(); i++)
    sum+=pow(abs(src.entry(i)),degree);
  return pow(sum,1.0/degree);
}

double Martin::minkowsky_distance (const ColumnVector& src1, const ColumnVector& src2, double degree) throw (invalid_argument) { 
  if (degree<=0)
    throw invalid_argument ("invalid parameter in Martin::minkowsky_distance");
  if (src1.number_rows()!=src2.number_rows())
    throw invalid_argument("mismatch in vector dimension in Martin::minkowsky_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_rows(); i++)
    sum+=pow(abs(src1.entry(i)-src2.entry(i)),degree);
  return pow(sum,1.0/degree);
}

double Martin::minkowsky_norm (const RowVector& src, double degree) throw (invalid_argument) { 
  if (degree<=0)
    throw invalid_argument ("invalid parameter in Martin::minkowsky_norm");
  double sum=0;
  for (unsigned int i=1; i<=src.number_columns(); i++)
    sum+=pow(abs(src.entry(i)),degree);
  return pow(sum,1.0/degree);
}

double Martin::minkowsky_distance (const RowVector& src1, const RowVector& src2, double degree) throw (invalid_argument) { 
  if (degree<=0)
    throw invalid_argument ("invalid parameter in Martin::minkowsky_distance");
  if (src1.number_columns()!=src2.number_columns())
    throw invalid_argument("mismatch in vector dimension in Martin::minkowsky_distance");
  double sum=0;
  for (unsigned int i=1; i<=src1.number_columns(); i++)
    sum+=pow(abs(src1.entry(i)-src2.entry(i)),degree);
  return pow(sum,1.0/degree);
}
