
#include "TacticsBoard.h"

using namespace Tribots;
using namespace std;

std::string& TacticsBoard::operator[]( const std::string& key ) {
  return std::map<std::string,std::string>::operator[] (key);
}

const std::string& TacticsBoard::operator[]( const std::string& key ) const {
  std::map<std::string, std::string>::const_iterator it = find (key);
  if (it==end())
    return emptystring;
  else
    return it->second;
}

const std::string& TacticsBoard::operator[]( const char* key ) const {
  return operator[] (std::string(key));
}
