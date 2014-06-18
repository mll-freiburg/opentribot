
#include "GyroDataReadWriter.h"
#include "../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;


GyroDataWriter::GyroDataWriter (std::ostream& d) throw () : dest(d) {;}

GyroDataWriter::~GyroDataWriter () throw () {
  dest << std::flush;
}

void GyroDataWriter::write (unsigned long int tav, unsigned long int t1, const GyroData& gd) throw () {
  dest.precision (5);
  dest << tav << '\t'
       << t1 << '\t'
      << gd.vrot << '\n';
}


GyroDataReader::GyroDataReader (std::istream& s) throw () : src(s), next(0) {
  if (src.good()) src >> next;
}

bool GyroDataReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool GyroDataReader::read_until (unsigned long int& tav, unsigned long int& t1, GyroData& gd, unsigned long int tuntil) throw () {
  tav=next;
  bool read_anything = false;
  while (!src.eof() && next<=tuntil) {
    tav=next;
    string line;
    vector<string> parts;
    getline (src, line);
    split_string (parts, line);
    src >> next;

    if (parts.size()>=2) {
      string2ulint (t1, parts[0]);
      string2double (gd.vrot, parts[1]);
      read_anything=true;
    }
  }
  return read_anything;
}
