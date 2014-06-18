#include <utility>
#include <algorithm>
#include "ObstacleLocation.h"
#include <limits>


using namespace Tribots;
using namespace std;

ObstacleDescriptor::ObstacleDescriptor () throw () :
    pos(0,0),
    width(0),
    player(-1),
    velocity(0,0) {;}

ObstacleDescriptor::ObstacleDescriptor (const ObstacleDescriptor& s) throw () :
    pos(s.pos),
    width(s.width),
    player(s.player),
    velocity(s.velocity) {;}

const ObstacleDescriptor& ObstacleDescriptor::operator= (const ObstacleDescriptor& s) throw () {
  pos=s.pos;
  width=s.width;
  player=s.player;
  velocity=s.velocity;
  return *this;
}

ObstacleLocation::ObstacleLocation (unsigned int n) throw (std::bad_alloc) :
    vector<ObstacleDescriptor> (n) {;}

ObstacleLocation::ObstacleLocation (const ObstacleLocation& ol) throw (std::bad_alloc) :
    vector<ObstacleDescriptor> (ol) {;}

void ObstacleLocation::writeAt(std::ostream &stream) const
{
  for (unsigned int i=0; i<size(); i++)
    stream << "("<< operator[](i).pos.x << "," << operator[](i).pos.y << ";" << operator[](i).width << ")";
}

int ObstacleLocation::readFrom(std::istream &stream)
{
  clear();
  do {
    ObstacleDescriptor d;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),'(');
    stream >> d.pos.x;
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),',');
    stream >> d.pos.y;
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),';');
    stream >> d.width;
    if (stream.fail() || stream.eof()) break;
    stream.ignore(std::numeric_limits<std::streamsize>::max(),')');

    push_back(d);
  } while (!(stream.fail() || stream.eof()));

  return size();
}
