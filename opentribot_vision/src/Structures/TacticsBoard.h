
#ifndef _Tribots_TacticsBoard_h_
#define _Tribots_TacticsBoard_h_

#include <map>
#include <string>

namespace Tribots {

  /** Taktikboard: Liste taktischer Einstellungen als Schluessel-Wert Paare;
    Zugriff mit dem []-Operator: wert = [key] */
  class TacticsBoard : public std::map<std::string, std::string> {
  public:
    std::string& operator[] (const std::string&);
    const std::string& operator[] (const std::string&) const;
    const std::string& operator[] (const char*) const;
  protected:
    std::string emptystring;
  };
    
}

#endif
