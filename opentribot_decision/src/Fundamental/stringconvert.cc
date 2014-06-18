
#include "stringconvert.h"
#include <cctype>
#include <cstdio>
#include "stdlib.h"
using namespace std;

void Tribots::split_string (vector<string>& dest, const string& src) throw (std::bad_alloc) {
  dest.clear();
  if (src.length()==0)
    return;
  int i=-1;
  bool was_white = isspace (src[0]);
  for (unsigned int j=0; j<src.length(); j++) {
    if (isspace (src[j])) {
      if (was_white) {
        i=j;
      } else {
        was_white=true;
        dest.push_back (src.substr (i+1, j-i-1));
        i=j;
      }
    } else {
      was_white=false;
    }
  }
  if (!was_white) {
    dest.push_back (src.substr (i+1, src.length()));
  }
}

bool Tribots::prefix (const std::string& p, const std::string& w) throw () {
  const unsigned int n=p.length();
  if (n>w.length())
    return false;
  for (unsigned int i=0; i<n; i++)
    if (p[i]!=w[i])
      return false;
  return true;
}

bool Tribots::string2double (double& dest, const string& src) throw () {
  if (src.length()>50) {
    dest=0.0;
    return false;
  }
  char* end_char;
  dest = strtod (src.c_str(), &end_char);
  if ((*end_char)!='\0') {
    dest=0.0;
    return false;
  }
  return true;
}

bool Tribots::string2float (float& dest, const string& src) throw () {
  double d;
  bool success = Tribots::string2double (d, src);
  dest=d;
  return success;
}

bool Tribots::string2lint (long int& d, const string& s) throw () {
  if (s.length()>50) {
    d=0;
    return false;
  }
  char* end_char;
  d = strtol (s.c_str(), &end_char, 0);
  if ((*end_char)!='\0') {
    d=0;
    return false;
  }
  return true;
}

bool Tribots::string2ulint (unsigned long int& d, const string& s) throw () {
  if (s.length()>50) {
    d=0;
    return false;
  }
  char* end_char;
  d = strtoul (s.c_str(), &end_char, 0);
  if ((*end_char)!='\0') {
    d=0;
    return false;
  }
  if ((s[0]=='-') && (d!=0)) {
    d=0;
    return false;
  }
  return true;
}

bool Tribots::string2int (int& d, const string& s) throw () {
  long int l;
  bool success = Tribots::string2lint (l, s);
  d=l;
  return success;
}

bool Tribots::string2uint (unsigned int& d, const string& s) throw () {
  unsigned long int l;
  bool success = Tribots::string2ulint (l, s);
  d=l;
  return success;
}

bool Tribots::string2bool (bool& d, const string& s) throw () {
  if (s==string("0")) {
    d=false;
    return true;
  }
  if (s==string("1")) {
    d=true;
    return true;
  }
  if (s==string("true")) {
    d=true;
    return true;
  }
  if (s==string("false")) {
    d=false;
    return true;
  }
  return false;
}

std::string Tribots::get_filepath (const std::string& src) throw (std::bad_alloc) {
  if (src.length()==0) return src;
  unsigned int pos=src.length()-1;
  while (pos>0 && src[pos]!='/')
    pos--;
  if (pos==0) {
    if (src[0]=='/')
      return string ("/");
    else
      return string ("");
  }
  return src.substr (0, pos+1);
}
  
std::string Tribots::remove_filepath (const std::string& src) throw (std::bad_alloc) {
  if (src.length()==0) return src;
  unsigned int pos=src.length()-1;
  while (pos>0 && src[pos]!='/')
    pos--;
  if (pos==0) {
    if (src[0]=='/')
      return src.substr(1, src.length()-1);
    else
      return src;
  }
  return src.substr(pos+1, src.length()-pos-1);
}

string Tribots::get_fileextension (const string& src) throw (bad_alloc) {
  if (src.length()==0) return src;
  unsigned int pos=src.length()-1;
  while (pos>0 && src[pos]!='.' && src[pos]!='/')
    pos--;
  if (pos==0) {
    if (src[0]=='.')
      return src;
    else
      return string ("");
  }
  if (src[pos]=='.')
    return src.substr (pos, src.length()-pos);
  else
    return string ("");
}

string Tribots::remove_fileextension (const string& src) throw (bad_alloc) {
  if (src.length()==0) return src;
  unsigned int pos=src.length()-1;
  while (pos>0 && src[pos]!='.' && src[pos]!='/')
    pos--;
  if (pos==0) {
    if (src[0]=='.')
      return string ("");
    else
      return src;
  }
  if (src[pos]=='.')
    return src.substr (0, pos);
  else
    return src;
}
