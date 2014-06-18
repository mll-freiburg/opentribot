/*

   1. defines the > and < operator for basic types and istream/ostream to 
   have similar behavior as >> and << but for binary i/o.
   
   For (some) types in the standard library (vector, map) I will define 
   these operators here as well. This is not quite clean, because 
   the vector and map headers will always be included via this header
   will thus pollute all other headers that include WTR_BinaryIO.h as well.  
   The (cleaner) alternative is to create a separate file for each 
   standard header to be included instead of the standard header and supply 
   our additions there. That may happen in the future...


   USAGE:

   include "WTR_BinaryIO.h"

   ..
   ofstream a("test");
   int      i;
   a < i < 2*i;
   ...
   a < myclass;  // exists if 'myclass.save(a)' is provided


   BUGS/PROBLEMS:
   - long long int's, long double's and similar non-standard types are 
   not supported,
   - of course binary format specification can be tricked by writing one 
   type and reading another, so be aware what you do when using things like 
   uint32_t or the like. 
*/

#ifndef BV_BINARYIO_H
#define BV_BINARYIO_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include "bvdebug.h"

using namespace std;

/** the following definitions will be used to overload operators < and > for
    binary read/write
*/

# define  _BV_BINARYOUT(CALLTYPE) \
inline std::ostream &\
operator<(std::ostream &o, const CALLTYPE &a) \
{\
  BV_DEBUGINFO5( "operator<(std::ostream &o, const CALLTYPE &a) try to invoke ostream::write for writing [" << sizeof(CALLTYPE) << "] bytes" ); \
  o.write( (char *) &a, sizeof(CALLTYPE) );\
  return o;\
}
 
# define  _BV_BINARYIN(CALLTYPE) \
inline std::istream &\
operator>(std::istream &i, CALLTYPE &a) \
{\
  BV_DEBUGINFO5( "operator>(std::istream &i, CALLTYPE &a) try to invoke istream::read for reading [" << sizeof(CALLTYPE) << "] bytes" ); \
  i.read( (char *) &a, sizeof(CALLTYPE) );\
  return i;\
}


# define _BV_BINARYOUT_CHAR(TYPE) _BV_BINARYOUT(TYPE)
# define _BV_BINARYIN_CHAR(TYPE) _BV_BINARYIN(TYPE)
 
namespace std {

/** floating point types */

_BV_BINARYOUT(double)
_BV_BINARYOUT(float)
 
_BV_BINARYIN(double)
_BV_BINARYIN(float)
 
/** integer types (signed is implicit) */
 
_BV_BINARYOUT(int)
_BV_BINARYOUT(unsigned)
_BV_BINARYOUT(short)
_BV_BINARYOUT(unsigned short)
_BV_BINARYOUT(long)
_BV_BINARYOUT(unsigned long)
_BV_BINARYOUT(bool)
 
_BV_BINARYIN(int)
_BV_BINARYIN(unsigned)
_BV_BINARYIN(short)
_BV_BINARYIN(unsigned short)
_BV_BINARYIN(long)
_BV_BINARYIN(unsigned long)
_BV_BINARYIN(bool)
 
/** character types - whether a char behaves signed or
    unsigned is implementation dependent,  pointers to
    signed, unsigned, and plain chars are possibly different entities
    so we need all three types here
*/
 
_BV_BINARYOUT_CHAR(char)
_BV_BINARYOUT_CHAR(signed char)
_BV_BINARYOUT_CHAR(unsigned char)
 
_BV_BINARYIN_CHAR(char)
_BV_BINARYIN_CHAR(signed char)
_BV_BINARYIN_CHAR(unsigned char)
 
}  // close std
 
#undef _BV_BINARYOUT
#undef _BV_BINARYIN
#undef _BV_BINARYOUT_CHAR
#undef _BV_BINARYIN_CHAR

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---vector<int>-------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
/** write a string to stream
    first specify length of the string, and write it o stream
    then write the string data to the string 
*/
inline std::ostream &
operator<(std::ostream &out, const std::string &s) {

  std::string::size_type  n; 
  n = s.size();
  BV_DEBUGINFO5( "operator<(std::ostream &out, const std::string &s) try to write the integer [" << n << "],  as string length" );
  out < n;
  BV_DEBUGINFO5( "operator<(std::ostream &out, const std::string &s) try to write [" << n << "] bytes as string chars" );    

  return out.write(s.data(),n);
}
/** read a string from stream
    first read an integer which is the string length
    then create a buffer with this length
    then read the string data in to this buffer
    but take care when reading the string data 
    I mean consider the zeros in the data ......
*/
inline std::istream &
operator>(std::istream &in, std::string &s) {

  std::string::size_type  n;

  BV_DEBUGINFO5( "operator>(std::istream &in, std::string &s) try to read an integer (string length)" );
  in > n;

  //BV_DEBUGINFO("operator > string : length=[" << n << "]");

  // FIXME : checking the EOF is enough ??????
  if(!in.eof()){

    // std::string::value_type *buffer = new std::string::value_type[n+1];

    std::string::value_type *buffer = new std::string::value_type[n];
    for(unsigned int i=0; i< n ;i++) buffer[i]=0;

    BV_DEBUGINFO5( "operator>(std::istream &in, std::string &s) try to read string chars, [" << n << "] bytes" );    
    in.read(buffer,n);

    // I want to copy buffer in the  std::string &s
    // but simply I cannot do the following
    //
    //    buffer[n] = 0;
    //    s = std::string(buffer);
    //
    // it will not work. 
    // because in the case that buffer contains elements equal to zero (0)
    // s = std::string(buffer) will catch the first 0 as NULL trmination sign.
    // so I will do as follows:

    BV_DEBUGINFO5( "operator>(std::istream &in, std::string &s) try to invoke operator<(std::ostream &o, const char& a) [" << n << "] times" ); 
    stringstream ss;
    for(unsigned int i=0; i< n ;i++){
      ss < buffer[i];
    }
    s = ss.str();
 
    delete buffer;
  }
  return in;
}





// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---vector<string>----------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
inline std::ostream &
operator<(std::ostream &out, vector<string> &vec) {

  out < vec.size();

  for( vector<string>::iterator it=vec.begin() ; it!=vec.end(); it++ ){
    out < *it;
  }
  return out;
}
inline std::istream &
operator>(std::istream &in, vector<string> &vec ) {
  unsigned int s; 
  in > s;
  for( int i=0;i<s;i++ ){
    string x;
    in > x ;
    vec.push_back( x);
  }
  return in;
}
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---vector<int>-------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
inline std::ostream &
operator<(std::ostream &out, vector<int> &vec) {

  out < vec.size();

  for( vector<int>::iterator it=vec.begin() ; it!=vec.end(); it++ ){
    out < *it;
  }
  return out;
}

inline std::istream &
operator>(std::istream &in, vector<int> &vec ) {
  unsigned int s; 
  in > s;
  for( int i=0;i<s;i++ ){
    int x;
    in > x ;
    vec.push_back( x);
  }
  return in;
}
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---vector<double>----------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
inline std::ostream &
operator<(std::ostream &out, vector<double> &vec) {

  out < vec.size();

  for( vector<double>::iterator it=vec.begin() ; it!=vec.end(); it++ ){
    out < *it;
  }
  return out;
}

inline std::istream &
operator>(std::istream &in, vector<double> &vec ) {
  unsigned int s; 
  in > s;
  for( int i=0;i<s;i++ ){
    double x;
    in > x ;
    vec.push_back( x);
  }
  return in;
}
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---vector<bool>------------------------------------------------------------
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
inline std::ostream &
operator<(std::ostream &out, vector<bool> &vec) {

  out < vec.size();

  for( vector<bool>::iterator it=vec.begin() ; it!=vec.end(); it++ ){
    out < *it;
  }
  return out;
}

inline std::istream &
operator>(std::istream &in, vector<bool> &vec ) {
  unsigned int s; 
  in > s;
  for( int i=0;i<s;i++ ){
    bool x;
    in > x ;
    vec.push_back( x);
  }
  return in;
}

#endif












