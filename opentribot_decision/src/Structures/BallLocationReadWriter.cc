
#include "BallLocationReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"

using namespace Tribots;
using namespace std;


BallLocationWriter::BallLocationWriter (std::ostream& d, bool b) throw () : dest(d), binary_mode(b) {
  if (binary_mode) {
    dest << "b2x";
  }
}

BallLocationWriter::~BallLocationWriter () throw () {
  dest << std::flush;
}

void BallLocationWriter::write (unsigned long int tav, unsigned long int t1, const BallLocation& bloc_vis, unsigned long int t2, const BallLocation& bloc_exec) throw () {
  if (binary_mode) {
    wunsigned3(dest,tav);
    wunsigned2(dest,tav-t1);
    wunsigned2(dest,t2-tav);

    wsigned2(dest,bloc_vis.pos.x/2);
    wsigned2(dest,bloc_vis.pos.y/2);
    wsigned2(dest,bloc_vis.pos.z/2);
    wsigned2(dest,bloc_vis.velocity.x*1000);
    wsigned2(dest,bloc_vis.velocity.y*1000);
    wsigned2(dest,bloc_vis.velocity.z*1000);

    wsigned2(dest,bloc_exec.pos.x/2);
    wsigned2(dest,bloc_exec.pos.y/2);
    wsigned2(dest,bloc_exec.pos.z/2);
    wsigned2(dest,bloc_exec.velocity.x*1000);
    wsigned2(dest,bloc_exec.velocity.y*1000);
    wsigned2(dest,bloc_exec.velocity.z*1000);

    wunsigned3(dest,static_cast<unsigned long int>(bloc_vis.lastly_seen.get_msec()));
    unsigned int knownattr = static_cast<unsigned char>(bloc_vis.pos_known)<<4;
    knownattr |= static_cast<unsigned char>(bloc_exec.pos_known);
    if (bloc_vis.velocity_known) knownattr |=128;
    if (bloc_exec.velocity_known) knownattr |=8;
    wunsigned1(dest,knownattr);
  } else {
    dest.precision (5);
    dest << tav << '\t'
        << t1 << '\t'
        << bloc_vis.pos.x << '\t' 
        << bloc_vis.pos.y << '\t' 
        << bloc_vis.pos.z << '\t' 
        << bloc_vis.velocity.x << '\t' 
        << bloc_vis.velocity.y << '\t' 
        << bloc_vis.velocity.z << '\t' 
        << bloc_vis.velocity_known << '\t'
        << bloc_vis.pos_known << '\t' 
        << bloc_vis.lastly_seen << '\t'
        << t2 << '\t'
        << bloc_exec.pos.x << '\t' 
        << bloc_exec.pos.y << '\t' 
        << bloc_exec.pos.z << '\t' 
        << bloc_exec.velocity.x << '\t' 
        << bloc_exec.velocity.y << '\t' 
        << bloc_exec.velocity.z << '\t' 
        << bloc_exec.velocity_known << '\t'
        << bloc_exec.pos_known << '\t' 
        << bloc_exec.lastly_seen << '\n';
  }
}


BallLocationReader::BallLocationReader (std::istream& s) throw () : src(s), next(0), encoding_type(0) {
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

bool BallLocationReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool BallLocationReader::read_until (unsigned long int& tav, unsigned long int& t1, BallLocation& bloc_vis, unsigned long int& t2, BallLocation& bloc_exec, unsigned long int tuntil) throw () {
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
      if (parts.size()>=5 && parts.size()<=7) {
        // altes Format mit nur einem Zeitstempel
        string2double (bloc_vis.pos.x, parts[0]);
        string2double (bloc_vis.pos.y, parts[1]);
        string2double (bloc_vis.velocity.x, parts[2]);
        string2double (bloc_vis.velocity.y, parts[3]);
        double d=0;
        string2double (d, parts[4]);
        bloc_vis.velocity_known = (d!=0);
        if (parts.size()==7) {
          int pk=0;
          string2int (pk, parts[5]);
          bloc_vis.pos_known = BallLocation::BallAttribute (pk);
          unsigned long int trans;
          string2ulint (trans, parts[6]);
          bloc_vis.lastly_seen.set_msec (trans);
        } else {
          bloc_vis.pos_known = BallLocation::known;
        }
        bloc_exec=bloc_vis;
        t1=t2=tav;
        read_anything=true;
      } else if (parts.size()==16) {
        // neues Format mit zwei Positionen und insgesamt 3 Zeitstempeln
        unsigned long int trans;
        string2ulint (t1, parts[0]);
        string2double (bloc_vis.pos.x, parts[1]);
        string2double (bloc_vis.pos.y, parts[2]);
        bloc_vis.pos.z=0;
        string2double (bloc_vis.velocity.x, parts[3]);
        string2double (bloc_vis.velocity.y, parts[4]);
        bloc_vis.velocity.z=0;
        double d=0;
        string2double (d, parts[5]);
        bloc_vis.velocity_known = (d!=0);
        int pk=0;
        string2int (pk, parts[6]);
        bloc_vis.pos_known = BallLocation::BallAttribute (pk);
        string2ulint (trans, parts[7]);
        bloc_vis.lastly_seen.set_msec (trans);
        string2ulint (t2, parts[8]);
        string2double (bloc_exec.pos.x, parts[9]);
        string2double (bloc_exec.pos.y, parts[10]);
        bloc_exec.pos.z=0;
        string2double (bloc_exec.velocity.x, parts[11]);
        string2double (bloc_exec.velocity.y, parts[12]);
        bloc_exec.velocity.z=0;
        d=0;
        string2double (d, parts[13]);
        bloc_exec.velocity_known = (d!=0);
        string2int (pk, parts[14]);
        bloc_exec.pos_known = BallLocation::BallAttribute (pk);
        string2ulint (trans, parts[15]);
        bloc_exec.lastly_seen.set_msec (trans);
        read_anything=true;
      } else if (parts.size()==20) {
        // neues Format mit 3D-Koordinaten
        unsigned long int trans;
        string2ulint (t1, parts[0]);
        string2double (bloc_vis.pos.x, parts[1]);
        string2double (bloc_vis.pos.y, parts[2]);
        string2double (bloc_vis.pos.z, parts[3]);
        string2double (bloc_vis.velocity.x, parts[4]);
        string2double (bloc_vis.velocity.y, parts[5]);
        string2double (bloc_vis.velocity.z, parts[6]);
        double d=0;
        string2double (d, parts[7]);
        bloc_vis.velocity_known = (d!=0);
        int pk;
        string2int (pk, parts[8]);
        bloc_vis.pos_known = BallLocation::BallAttribute (pk);
        string2ulint (trans, parts[9]);
        bloc_vis.lastly_seen.set_msec (trans);
        string2ulint (t2, parts[10]);
        string2double (bloc_exec.pos.x, parts[11]);
        string2double (bloc_exec.pos.y, parts[12]);
        string2double (bloc_exec.pos.z, parts[13]);
        string2double (bloc_exec.velocity.x, parts[14]);
        string2double (bloc_exec.velocity.y, parts[15]);
        string2double (bloc_exec.velocity.z, parts[16]);
        d=0;
        string2double (d, parts[17]);
        bloc_exec.velocity_known = (d!=0);
        string2int (pk, parts[18]);
        bloc_exec.pos_known = BallLocation::BallAttribute (pk);
        string2ulint (trans, parts[19]);
        bloc_exec.lastly_seen.set_msec (trans);
        read_anything=true;
      }
    } else {
      // binaere Codierung
      t1=tav-runsigned2(src);
      t2=tav+runsigned2(src);

      bloc_vis.pos.x=2*rsigned2(src);
      bloc_vis.pos.y=2*rsigned2(src);
      bloc_vis.pos.z=2*rsigned2(src);
      bloc_vis.velocity.x=1e-3*rsigned2(src);
      bloc_vis.velocity.y=1e-3*rsigned2(src);
      bloc_vis.velocity.z=1e-3*rsigned2(src);
      if (encoding_type==1)
        runsigned1(src);  // hier stand mal die Qualitaet, ignorieren

      bloc_exec.pos.x=2*rsigned2(src);
      bloc_exec.pos.y=2*rsigned2(src);
      bloc_exec.pos.z=2*rsigned2(src);
      bloc_exec.velocity.x=1e-3*rsigned2(src);
      bloc_exec.velocity.y=1e-3*rsigned2(src);
      bloc_exec.velocity.z=1e-3*rsigned2(src);
      if (encoding_type==1)
        runsigned1(src);  // hier stand mal die Qualitaet, ignorieren

      unsigned long int ttt = runsigned3(src);
      bloc_vis.lastly_seen.set_msec(ttt);
      bloc_exec.lastly_seen.set_msec(ttt);
      unsigned char c = runsigned1 (src);
      bloc_vis.pos_known = BallLocation::BallAttribute ((c>>4)&0x07);
      bloc_exec.pos_known = BallLocation::BallAttribute (c & 0x07);
      bloc_vis.velocity_known = (c>>7);
      bloc_exec.velocity_known = ((c>>3)& 0x01);
      if (encoding_type==1)
        bloc_vis.velocity_known=bloc_exec.velocity_known=true;

      next = runsigned3(src);
      read_anything=true;
    }
  }
  return read_anything;
}
