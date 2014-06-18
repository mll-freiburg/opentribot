#include <limits>
#include "VisibleObject.h"


Tribots::VisibleObject::VisibleObject () throw () {
  pos.x=0;
  pos.y=0;
  object_type=VisibleObject::unknown;
  width=0;
}

Tribots::VisibleObject::VisibleObject (Vec pos1, ObjectType ot1, double w, double z1) throw () : pos(pos1), object_type(ot1), width(w), z(z1) {;}

Tribots::VisibleObject::VisibleObject (const VisibleObject& src) throw () : pos(src.pos), object_type(src.object_type), width(src.width), z(src.z) {;}

const Tribots::VisibleObject& Tribots::VisibleObject::operator= (const Tribots::VisibleObject& src) throw () {
  pos=src.pos;
  object_type=src.object_type;
  width=src.width;
  z=src.z;
  return (*this);
}

Tribots::VisibleObjectList::VisibleObjectList (const Tribots::VisibleObjectList& src) throw (std::bad_alloc) : timestamp (src.timestamp), objectlist (src.objectlist) {;}

const Tribots::VisibleObjectList& Tribots::VisibleObjectList::operator= (const Tribots::VisibleObjectList& src) throw (std::bad_alloc) {
  timestamp = src.timestamp;
  objectlist = src.objectlist;
  return *this;
}                                              

void Tribots::VisibleObjectList::writeAt(std::ostream &stream) const
{
  for (unsigned int i=0; i< objectlist.size(); i++)
    stream << "("<< objectlist[i].pos.x << "," << objectlist[i].pos.y << ";" 
	   << objectlist[i].width << "," << objectlist[i].object_type <<  ")";
}

int Tribots::VisibleObjectList::readFrom(std::istream &stream)
{
  int n=0;
  objectlist.clear();
 
  do {
    Tribots::VisibleObject o;
    
    stream.ignore(std::numeric_limits<std::streamsize>::max(),'(');
    stream >> o.pos.x;
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),',');
    stream >> o.pos.y;
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),';');
    stream >> o.width;
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),',');
    int ot;
    stream >> ot;
    o.object_type = VisibleObject::ObjectType (ot);
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),')');
    
    objectlist.push_back(o);
    n++;
  } while (!(stream.fail() || stream.eof()));
  
  return n;
}
