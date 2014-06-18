
#include "VisibleObjectReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"

using namespace Tribots;
using namespace std;


VisibleObjectWriter::VisibleObjectWriter (std::ostream& d, bool b) throw () : dest(d), binary_mode(b) {
  if (binary_mode) {
    dest << "b1x";
  }
  old_trec=old_tav=0;
}

VisibleObjectWriter::~VisibleObjectWriter () throw () {
  dest << std::flush;
}

void VisibleObjectWriter::write (unsigned long int tav, unsigned long int trec, const VisibleObject& obj, unsigned int id) throw () {
  if (binary_mode) {
    if (old_trec!=trec || old_tav!=tav) {
      // neue Zeitstempel
      wunsigned1 (dest,static_cast<unsigned long int>(0x80 | obj.object_type<<2 | id));
      wunsigned3 (dest,tav);
      wunsigned2 (dest,tav-trec);
    } else {
      // alte Zeitstempel gelten weiter
      wunsigned1 (dest,obj.object_type<<2 | id);      
    }
    wsigned2 (dest,obj.pos.x/2);
    wsigned2 (dest,obj.pos.y/2);
    wunsigned2 (dest,(obj.object_type==VisibleObject::ball3d ? obj.z : obj.width));
  } else {
    dest.precision (5);
    dest << tav << '\t' 
         << trec << '\t' 
         << obj.pos.x << '\t' 
         << obj.pos.y << '\t' 
         << obj.object_type << '\t' 
         << (obj.object_type==VisibleObject::ball3d ? obj.z : obj.width);
    if (id!=0) {
      dest << '\t' << id;
    }
    dest << '\n';
  }
  old_trec=trec;
  old_tav=tav;
}

void VisibleObjectWriter::write (unsigned long int tav, unsigned long int trec, const VisibleObjectList& obj, unsigned int id) throw () {
  dest.precision (5);
  for (unsigned int i=0; i<obj.objectlist.size(); i++)
    write (tav, trec, obj.objectlist[i], id);
}


VisibleObjectReader::VisibleObjectReader (std::istream& s) throw () : src(s), next(0), encoding_type (0) {
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
  old_tav=old_trec=0;
  if (encoding_type>=1) {
    next = binary_read_old ();
  } else {
    src >> next;
  }
}

bool VisibleObjectReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool VisibleObjectReader::read_until (unsigned long int& tav, unsigned long int& trec, vector<VisibleObjectList>& obj, unsigned long int tuntil) throw () {
  tav=next;
  bool read_anything = false;
  unsigned int id;
  for (unsigned int i=0; i<obj.size(); i++)
    obj[i].objectlist.clear();
  while (!src.eof() && next<=tuntil) {
    if (tav!=next) {
      for (unsigned int i=0; i<obj.size(); i++)
        obj[i].objectlist.clear();
      tav=next;
    }
    VisibleObject dest;
    if (encoding_type==0) {
      // ascii-Codierung
      string line;
      vector<string> parts;
      getline (src, line);
      split_string (parts, line);
      src >> next;
      if (parts.size()>=5) {
        string2ulint (trec, parts[0]);
        string2double (dest.pos.x, parts[1]);
        string2double (dest.pos.y, parts[2]);
        int ot;
        string2int (ot, parts[3]);
        dest.object_type = VisibleObject::ObjectType (ot);
        string2double (dest.width, parts[4]);
        dest.z=0;
        if (dest.object_type==VisibleObject::ball3d) {
          dest.z=dest.width;
          dest.width=0;
        }
        if (parts.size()>=6)
          string2uint (id, parts[5]);
        else
          id=0;
        read_anything=true;
      }
    } else {
      // binaere Codierung
      trec = old_trec;
      tav = old_tav;
      dest.object_type = old_objecttype;
      id = old_id;
      dest.pos.x=2*rsigned2 (src);
      dest.pos.y=2*rsigned2 (src);
      if (dest.object_type==VisibleObject::ball3d) {
        dest.width=0;
        dest.z=runsigned2 (src);
      } else {
        dest.z=0;
        dest.width=runsigned2 (src);
      }
      read_anything=true;
      next=binary_read_old ();
    }
    if (read_anything) {
      while (id>=obj.size()) {
        VisibleObjectList emptylist;
        obj.push_back (emptylist);
      }
      if (obj[id].objectlist.size()==0)
        obj[id].timestamp.set_msec(trec);
      obj[id].objectlist.push_back (dest);
    }
  }
  return read_anything;
}

unsigned long int VisibleObjectReader::binary_read_old () {
  src.peek();
  if (src.eof())
    return 0;
  unsigned char c = runsigned1 (src);
  old_objecttype = VisibleObject::ObjectType ((c>>2)&0x7f);
  old_id = c & 0x03;
  if (c & 0x80) {
    old_tav = runsigned3 (src);
    old_trec = old_tav-runsigned2 (src);
  }
  return old_tav;
}
