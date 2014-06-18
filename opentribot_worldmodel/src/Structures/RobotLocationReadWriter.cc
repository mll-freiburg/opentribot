
#include "RobotLocationReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"

using namespace Tribots;
using namespace std;


RobotLocationWriter::RobotLocationWriter (std::ostream& d, bool b) throw () : dest(d), binary_mode(b) {
  if (binary_mode) {
    dest << "b1x";
  }
}

RobotLocationWriter::~RobotLocationWriter () throw () {
  dest << std::flush;
}

void RobotLocationWriter::write (unsigned long int tav, unsigned long int t1, const RobotLocation& rloc_vis, unsigned long int t2, const RobotLocation& rloc_exec) throw () {
  if (binary_mode) {
    wunsigned3(dest,tav);
    wunsigned2(dest,tav-t1);
    wunsigned2(dest,t2-tav);
    
    wsigned2(dest,rloc_vis.pos.x/2);
    wsigned2(dest,rloc_vis.pos.y/2);
    wunsigned2(dest,rloc_vis.heading.get_rad()*1000);
    wsigned2(dest,rloc_vis.vtrans.x*1000);
    wsigned2(dest,rloc_vis.vtrans.y*1000);
    wsigned1(dest,rloc_vis.vrot*10);
    
    wsigned2(dest,rloc_exec.pos.x/2);
    wsigned2(dest,rloc_exec.pos.y/2);
    wunsigned2(dest,rloc_exec.heading.get_rad()*1000);
    wsigned2(dest,rloc_exec.vtrans.x*1000);
    wsigned2(dest,rloc_exec.vtrans.y*1000);
    wsigned1(dest,rloc_exec.vrot*10);
    
    wunsigned1(dest,static_cast<unsigned long int>((static_cast<unsigned char>(rloc_vis.kick)<<7) | (static_cast<unsigned char>(rloc_exec.kick)<<6) | (static_cast<unsigned char>(rloc_vis.stuck.robot_stuck)<<5) | (static_cast<unsigned char>(!rloc_vis.valid<<4))));
  } else {
    dest.precision (5);
    dest << tav << '\t'
        << t1 << '\t'
        << rloc_vis.pos.x << '\t' 
        << rloc_vis.pos.y << '\t' 
        << rloc_vis.heading.get_rad() << '\t' 
        << rloc_vis.vtrans.x << '\t' 
        << rloc_vis.vtrans.y << '\t' 
        << rloc_vis.vrot << '\t' 
        << rloc_vis.kick << '\t'
        << t2 << '\t'
        << rloc_exec.pos.x << '\t' 
        << rloc_exec.pos.y << '\t' 
        << rloc_exec.heading.get_rad() << '\t' 
        << rloc_exec.vtrans.x << '\t' 
        << rloc_exec.vtrans.y << '\t' 
        << rloc_exec.vrot << '\t' 
        << rloc_exec.kick << '\t'
        << rloc_vis.stuck.robot_stuck << '\n';
  }
}


RobotLocationReader::RobotLocationReader (std::istream& s) throw () : src(s), next(0), encoding_type(0) {
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

bool RobotLocationReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool RobotLocationReader::read_until (unsigned long int& tav, unsigned long int& t1, RobotLocation& rloc_vis, unsigned long int& t2, RobotLocation& rloc_exec, unsigned long int tuntil) throw () {
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
      if (parts.size()==7) {
        // altes Format mit nur einem Zeitstempel
        double trans;
        string2double (rloc_vis.pos.x, parts[0]);
        string2double (rloc_vis.pos.y, parts[1]);
        string2double (trans, parts[2]);
        rloc_vis.heading.set_rad (trans);
        string2double (rloc_vis.vtrans.x, parts[3]);
        string2double (rloc_vis.vtrans.y, parts[4]);
        string2double (rloc_vis.vrot, parts[5]);
        string2bool (rloc_vis.kick, parts[6]);
        rloc_vis.stuck.robot_stuck = false;
        rloc_exec=rloc_vis;
        t1=t2=tav;
        read_anything=true;
      } else if (parts.size()>=17) {
        // neues Format mit zwei Positionen und insgesamt 3 Zeitstempeln
        double trans;
        string2ulint (t1, parts[0]);
        string2double (rloc_vis.pos.x, parts[1]);
        string2double (rloc_vis.pos.y, parts[2]);
        string2double (trans, parts[3]);
        rloc_vis.heading.set_rad (trans);
        string2double (rloc_vis.vtrans.x, parts[4]);
        string2double (rloc_vis.vtrans.y, parts[5]);
        string2double (rloc_vis.vrot, parts[6]);
        string2bool (rloc_vis.kick, parts[7]);
        string2ulint (t2, parts[8]);
        string2double (rloc_exec.pos.x, parts[9]);
        string2double (rloc_exec.pos.y, parts[10]);
        string2double (trans, parts[11]);
        rloc_exec.heading.set_rad (trans);
        string2double (rloc_exec.vtrans.x, parts[12]);
        string2double (rloc_exec.vtrans.y, parts[13]);
        string2double (rloc_exec.vrot, parts[14]);
        string2bool (rloc_exec.kick, parts[15]);
        string2bool (rloc_vis.stuck.robot_stuck, parts[16]);
        rloc_exec.stuck.robot_stuck = rloc_vis.stuck.robot_stuck;
        read_anything=true;
      }
    } else {
      // binaere Codierung
      t1=tav-runsigned2(src);
      t2=tav+runsigned2(src);

      rloc_vis.pos.x=2*rsigned2(src);
      rloc_vis.pos.y=2*rsigned2(src);
      rloc_vis.heading.set_rad (static_cast<double>(runsigned2(src))/1000.0);
      rloc_vis.vtrans.x=1e-3*static_cast<double>(rsigned2(src));
      rloc_vis.vtrans.y=1e-3*static_cast<double>(rsigned2(src));
      rloc_vis.vrot=0.1*static_cast<double>(rsigned1(src));
      
      rloc_exec.pos.x=2*rsigned2(src);
      rloc_exec.pos.y=2*rsigned2(src);
      rloc_exec.heading.set_rad (static_cast<double>(runsigned2(src))/1000.0);
      rloc_exec.vtrans.x=1e-3*static_cast<double>(rsigned2(src));
      rloc_exec.vtrans.y=1e-3*static_cast<double>(rsigned2(src));
      rloc_exec.vrot=0.1*static_cast<double>(rsigned1(src));

      unsigned char c = runsigned1(src);
      rloc_vis.kick = c & 0x80;
      rloc_exec.kick = c & 0x40;
      rloc_vis.stuck.robot_stuck=rloc_exec.stuck.robot_stuck= c & 0x20;
      rloc_vis.valid=rloc_exec.valid=!(c & 0x10);

      next = runsigned3 (src);

      read_anything=true;
    }
  }
  return read_anything;
}
