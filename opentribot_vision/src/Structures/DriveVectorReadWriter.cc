
#include "DriveVectorReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"

using namespace Tribots;
using namespace std;


DriveVectorWriter::DriveVectorWriter (std::ostream& d, bool b) throw () : dest(d), binary_mode(b) {
  if (binary_mode) {
    dest << "b1x";
  }
}

DriveVectorWriter::~DriveVectorWriter () throw () {
  dest << std::flush;
}

void DriveVectorWriter::write (unsigned long int tav, unsigned long int trec, const DriveVector& obj) throw () {
  if (binary_mode) {
    wunsigned3(dest,tav);
    wunsigned2(dest,tav-trec);
    wsigned2(dest,obj.vtrans.x*1000);
    wsigned2(dest,obj.vtrans.y*1000);
    wsigned1(dest,obj.vrot*10);
    wunsigned1(dest,static_cast<unsigned long int>(static_cast<unsigned char>(obj.kick)<<7 | static_cast<unsigned char>(obj.mode)));
    wsigned2(dest,obj.vaux[0]*100);
    wsigned2(dest,obj.vaux[1]*100);
    wsigned2(dest,obj.vaux[2]*100);
  } else {
    dest.precision (5);
    dest << tav << '\t' 
        << trec << '\t'
        << obj.vtrans.x << '\t'
        << obj.vtrans.y << '\t'
        << obj.vrot << '\t'
        << obj.kick << '\t'
        << obj.mode << '\t'
        << obj.vaux[0] << '\t'
        << obj.vaux[1] << '\t'
        << obj.vaux[2] << '\n';
  }
}


DriveVectorReader::DriveVectorReader (std::istream& s) throw () : src(s), next(0), encoding_type(0) {
  if (src.good()) {
    if (src.peek()=='b') {
      // binaerer Modus, Version lesen
      char c;
      src.get(c);
      while (true) {
        src.get(c);
        if (c>='0' && c<='9') {
          encoding_type=encoding_type*10+(c-'0');
        } else {
          break;
        }
      }
    }
  }
  if (encoding_type>=1) {
    next = runsigned3(src);
  } else {
    src >> next;
  }
}

bool DriveVectorReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool DriveVectorReader::read_until (unsigned long int& tav, unsigned long int& trec, DriveVector& obj, unsigned long int tuntil) throw () {
  tav=next;
  bool read_anything = false;
  while (!src.eof() && next<=tuntil) {
    tav=next;
    if (encoding_type==0) {
      // ascii-Codierung
      string line;
      vector<string> parts;
      getline (src, line);
      split_string (parts, line);
      src >> next;
      if (parts.size()>4) {
        read_anything=true;
        string2ulint (trec, parts[0]);
        string2double (obj.vtrans.x, parts[1]);
        string2double (obj.vtrans.y, parts[2]);
        string2double (obj.vrot, parts[3]);
        string2uint (obj.kick, parts[4]);
      }
      if (parts.size()>7) {
        int mode;
        string2int (mode, parts[5]);
        obj.mode = DriveVectorMode (mode);
        string2double (obj.vaux[0], parts[6]);
        string2double (obj.vaux[1], parts[7]);
        string2double (obj.vaux[2], parts[8]);
      }
    } else {
      // binaere Codierung
      trec=tav-runsigned2(src);
      obj.vtrans.x=1e-3*rsigned2(src);
      obj.vtrans.y=1e-3*rsigned2(src);
      obj.vrot=1e-1*rsigned1(src);
      unsigned char c = runsigned1(src);
      obj.kick=c&0x80;
      obj.mode=DriveVectorMode(c&0x7f);
      obj.vaux[0]=1e-2*rsigned2(src);
      obj.vaux[1]=1e-2*rsigned2(src);
      obj.vaux[2]=1e-2*rsigned2(src);
      read_anything=true;

      next=runsigned3(src);
    }
  }
  return read_anything;
}
