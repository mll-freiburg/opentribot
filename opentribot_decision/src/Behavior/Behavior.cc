#include "Behavior.h"

void Tribots::Behavior::printHierarchy (std::ostream& os, unsigned int level) const throw () {
  for (unsigned int i=0; i<level; i++)
    os << "  ";
  os << getName () << '\n';
}
