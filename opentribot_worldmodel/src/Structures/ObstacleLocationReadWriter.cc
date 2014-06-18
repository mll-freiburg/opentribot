
#include "ObstacleLocationReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"

using namespace Tribots;
using namespace std;


ObstacleLocationWriter::ObstacleLocationWriter (std::ostream& d, bool b) throw () : dest(d), binary_mode (b) {
  if (binary_mode) {
    dest << "b2x";
  }
}

ObstacleLocationWriter::~ObstacleLocationWriter () throw () {
  dest << std::flush;
}

void ObstacleLocationWriter::write (unsigned long int tav, const ObstacleLocation& obj) throw () {
  vector<ObstacleDescriptor>::const_iterator it = obj.begin();
  vector<ObstacleDescriptor>::const_iterator itend = obj.end();
  dest.precision (5);
  unsigned long int first=0x80;
  while (it<itend) {
    if (binary_mode) {
      first = first | (it->player>=0 ? 0x40 : 0x00) | (it->player&0x3f);
      wunsigned1 (dest,first);
      if (first & 0x80) {
        wunsigned3 (dest,tav);
      }
      first=0x00;
      wsigned2 (dest,it->pos.x/2);
      wsigned2 (dest,it->pos.y/2);
      wsigned1 (dest,it->width/30);
      wsigned1 (dest,it->velocity.x*10);
      wsigned1 (dest,it->velocity.y*10);
    } else {
      dest << tav << '\t'
           << it->pos.x << '\t'
           << it->pos.y << '\t'
           << it->width << '\t'
           << it->player << '\t'
           << it->velocity.x << '\t'
           << it->velocity.y << '\n';
    }
    it++;
  }
}

ObstacleLocationReader::ObstacleLocationReader (std::istream& s) throw () : src(s), next(0), encoding_type(0) {
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
  old_tav=0;
  old_player=-1;
  if (encoding_type>=1) {
    next = binary_read_old ();
  } else {
    src >> next;
  }
}

bool ObstacleLocationReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool ObstacleLocationReader::read_until (unsigned long int& tav, ObstacleLocation& obj, unsigned long int tuntil) throw () {
  obj.clear();
  tav=next;
  bool read_anything = false;
  while (!src.eof() && next<=tuntil) {
    if (tav!=next) {
      obj.clear();
      tav=next;
    }
    if (encoding_type==0) {
      // ascii-Codierung
      string line;
      vector<string> parts;
      getline (src, line);
      split_string (parts, line);
      src >> next;
      ObstacleDescriptor od;
      if (parts.size()>=3) {
        string2double (od.pos.x, parts[0]);
        string2double (od.pos.y, parts[1]);
        string2double (od.width, parts[2]);
        if (parts.size()>=4)
          string2int (od.player, parts[3]);
        if (parts.size()>=6) {
          string2double (od.velocity.x, parts[4]);
          string2double (od.velocity.y, parts[5]);
        }
        obj.push_back (od);
        read_anything=true;
      }
    } else {
      // binaere Codierung
      tav = old_tav;
      ObstacleDescriptor od;
      od.pos.x = 2*rsigned2 (src);
      od.pos.y = 2*rsigned2 (src);
      od.width = 30*rsigned1(src);
      if (encoding_type>=2) {
        od.velocity.x = 0.1*static_cast<double>(rsigned1(src));
        od.velocity.y = 0.1*static_cast<double>(rsigned1(src));
      }
      od.player=old_player;
      obj.push_back (od);
      read_anything=true;
      next = binary_read_old();
    }
  }
  return read_anything;
}

unsigned long int ObstacleLocationReader::binary_read_old () {
  src.peek();
  if (src.eof())
    return 0;
  unsigned char c = runsigned1 (src);
  old_player = (c&0x40 ? c&0x3f : -1);
  if (c & 0x80) {
    old_tav = runsigned3 (src);
  }
  return old_tav;
}
