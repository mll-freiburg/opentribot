
#include "GameStateReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"

using namespace Tribots;
using namespace std;


GameStateWriter::GameStateWriter (std::ostream& d, bool b) throw () : dest(d), binary_mode(b) {
  if (binary_mode) {
    dest << "b1x";
  }
}

GameStateWriter::~GameStateWriter () throw () {
  dest << std::flush;
}

void GameStateWriter::write (unsigned long int tav, const GameState& obj, const char* ptype, const char* prole, const char* pbehavior) throw () {
  if (binary_mode) {
    wunsigned3(dest,tav);
    wunsigned1(dest,static_cast<unsigned int>(obj.refstate));
    wunsigned1(dest,static_cast<unsigned int>(obj.in_game));
    bool change=false;
    if (playertype==ptype) {
      wunsigned1(dest,0u);
    } else {
      playertype=ptype;
      wunsigned1(dest,playertype.size());
      for (unsigned int i=0; i<playertype.size(); i++)
        dest.put (playertype[i]);
      change=true;
    }
    if (playerrole==prole && !change) {
      wunsigned1(dest,0u);
    } else {
      playerrole=prole;
      wunsigned1(dest,playerrole.size());
      for (unsigned int i=0; i<playerrole.size(); i++)
        dest.put (playerrole[i]);
      change=true;
    }
    if (behavior==pbehavior && !change) {
      wunsigned2(dest,0u);
    } else {
      behavior=pbehavior;
      wunsigned2(dest,behavior.size());
      for (unsigned int i=0; i<behavior.size(); i++)
        dest.put (behavior[i]);
    }
  } else {
    playertype=ptype;
    playerrole=prole;
    behavior=pbehavior;
    if (playertype=="")
      playertype="-";
    if (playerrole=="")
      playerrole="-";
    if (behavior=="")
      behavior="-";
    dest << tav << '\t' 
        << obj.refstate << '\t'
        << obj.in_game << '\t'
        << playertype << '\t'
        << playerrole << '\t'
        << behavior << '\n';
  }
}


GameStateReader::GameStateReader (std::istream& s) throw () : src(s), next(0), encoding_type(0) {
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

bool GameStateReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool GameStateReader::read_until (unsigned long int& tav, GameState& obj, std::string& ptype, std::string& prole, std::string& pbehavior, unsigned long int tuntil) throw () {
  tav=next;
  bool read_anything = false;
  while (!src.eof() && next<=tuntil) {
    tav=next;
    if (encoding_type==0) {
      //ascii-Codierung
      string line;
      vector<string> parts;
      getline (src, line);
      split_string (parts, line);
      src >> next;
      if (parts.size()>=2) {
        read_anything=true;
        int a;
        string2int (a, parts[0]);
        obj.refstate = RefereeState (a);
        string2bool (obj.in_game, parts[1]);
      }
      if (parts.size()>=5) {
        playertype = parts[2];
        playerrole = parts[3];
        behavior = parts[4];
      }
      ptype=playertype;
      prole=playerrole;
      pbehavior=behavior;
    } else {
      // binaere Codierung
      obj.refstate = RefereeState (runsigned1(src));
      obj.in_game = runsigned1(src);
      unsigned int len;
      len = runsigned1 (src);
      if (len>0) {
        playertype.resize(len);
        for (unsigned int i=0; i<len; i++)
          playertype[i]=src.get ();
      }
      len = runsigned1 (src);
      if (len>0) {
        playerrole.resize(len);
        for (unsigned int i=0; i<len; i++)
          playerrole[i]=src.get ();
      }
      len = runsigned2 (src);
      if (len>0) {
        behavior.resize(len);
        for (unsigned int i=0; i<len; i++)
          behavior[i]=src.get ();
      }
      next = runsigned3(src);
      ptype=playertype;
      prole=playerrole;
      pbehavior=behavior;
      read_anything=true;
    }
  }
  return read_anything;
}
