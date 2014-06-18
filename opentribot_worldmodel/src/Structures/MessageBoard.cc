
#include "MessageBoard.h"
#include "../Fundamental/stringconvert.h"
#include "../WorldModel/WorldModel.h"  // nur fuer debug
#include <iostream>

#define DEBUG_INCOMING 0


Tribots::MessageBoard::MessageBoard () throw (std::bad_alloc) : incoming (50), outgoing (50), null_string ("") {
  clear();
}

Tribots::MessageBoard::MessageBoard (const MessageBoard& mb) throw (std::bad_alloc) : incoming (0), outgoing (0), null_string ("") {
  operator= (mb);
}

const Tribots::MessageBoard& Tribots::MessageBoard::operator= (const Tribots::MessageBoard& mb) throw (std::bad_alloc) {
  MessageBoard* nc = const_cast<MessageBoard*> (&mb);  // nicht die feine englische Art, aber Semantik-erhaltend
  nc->flush_outgoing ();
  incoming = mb.incoming;
  outgoing = mb.outgoing;
  return *this;
}

const std::vector<std::string> Tribots::MessageBoard::get_incoming () const throw () {
  return incoming;
}

void Tribots::MessageBoard::flush_outgoing () {
  outgoing_stream << std::flush;
  std::string line;
  while (!outgoing_stream.eof()) {
    std::getline (outgoing_stream, line);
    if (outgoing_stream.eof() && line.length()==0)
      break;
    outgoing.push_back (line);
  }
  outgoing_stream.clear();    // eof zuruecksetzen

  // Duplikate loeschen
  for (int line = outgoing.size()-1; line>=0; line--) {
    for (unsigned int i=line+1; i<outgoing.size(); i++) {
      if (outgoing[i]==outgoing[line]) {
        outgoing.erase (outgoing.begin()+line);
        break;
      }
    }
  }
}

const std::vector<std::string> Tribots::MessageBoard::get_outgoing () const throw () {
  MessageBoard* tc = const_cast<MessageBoard*>(this);  // nicht die feine englische Art, aber Semantik-erhaltend
  tc->flush_outgoing ();
  return outgoing;
}

void Tribots::MessageBoard::clear () throw () {
  incoming.clear();
  outgoing.clear();
}

void Tribots::MessageBoard::clear_incoming () throw () {
  incoming.clear();
}

void Tribots::MessageBoard::clear_outgoing () throw () {
  outgoing.clear();
}

void Tribots::MessageBoard::publish (const std::string& s) throw () {
  outgoing.push_back (s);
}

void Tribots::MessageBoard::publish (const std::vector<std::string>& s) throw () {
  outgoing.insert (outgoing.end(), s.begin(), s.end());
}

void Tribots::MessageBoard::publish (const char* s) throw () {
  outgoing.push_back (std::string(s));
}

std::ostream& Tribots::MessageBoard::publish_stream () throw () {
  return outgoing_stream;
}

void Tribots::MessageBoard::receive (const std::string& s) throw () {
  incoming.push_back (s);
#if DEBUG_INCOMING
  LOUT << "MBoard::receive: " << s << '\n';
#endif
}

void Tribots::MessageBoard::receive (const char* s) throw () {
  incoming.push_back (std::string(s));
#if DEBUG_INCOMING
  LOUT << "MBoard::receive: " << s << '\n';
#endif
}

void Tribots::MessageBoard::receive (const std::vector<std::string>& s) throw () {
  incoming.insert (incoming.end(), s.begin(), s.end());
#if DEBUG_INCOMING
  for (unsigned int i=0; i<s.size(); i++) {
    LOUT << "MBoard::receive: " << s[i] << '\n';
  }
#endif
}

const std::string& Tribots::MessageBoard::scan_for_prefix (const std::string& s) const throw () {
  if(incoming.size() > 0){
    std::vector<std::string>::const_iterator itbegin = incoming.begin();
    std::vector<std::string>::const_iterator it = incoming.end()-1;
    while (itbegin<=it) {
      if (Tribots::prefix (s, *it))
        return (*it);
      else
        it--;
    }
  }
  return null_string;
}

const std::string& Tribots::MessageBoard::scan_for_prefix (const char* s) const throw () {
  std::string s1 (s);
  return scan_for_prefix (s1);
}
