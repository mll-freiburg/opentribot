#include "MessageBoardReadWriter.h"
#include "../Fundamental/stringconvert.h"
#include "../Fundamental/binary_encoding.h"
#include <vector>
#include <stdlib.h>
#include <algorithm>
using namespace Tribots;
using namespace std;


MessageBoardWriter::MessageBoardWriter (std::ostream& d, bool) throw () : dest(d), binary_mode(true) {
  if (binary_mode) {
    dest << "b1x";
  }
}

MessageBoardWriter::~MessageBoardWriter () throw () {
  dest << std::flush;
}

void MessageBoardWriter::write (unsigned long int tav, const MessageBoard& mb) throw () {
  write (tav, mb.get_outgoing(), mb.get_incoming());
}

void MessageBoardWriter::matchMessages (
                                        char delChar,
                                        char insChar,
                                        std::vector<std::string>& oldList,
                                        const std::vector<std::string>& newList) {
  vector<bool> keepOld (oldList.size(), false);
  vector<bool> keepNew (newList.size(), false);
  for (unsigned int i=0; i<oldList.size(); i++) {
    vector<string>::const_iterator it = find (newList.begin(), newList.end(), oldList[i]);
    if (it!=newList.end()) {
      // Eintrag bleibt erhalten
      int idx = it-newList.begin();
      keepOld[i]=true;
      keepNew[idx]=true;
    }
  }
  for (unsigned int i=keepOld.size(); i!=0; i--) {
    if (!keepOld[i-1]) {
      dest.put (delChar);
      wunsigned1 (dest, i-1);
      oldList.erase (oldList.begin()+i-1);
    }
  }
  for (unsigned int i=0; i<keepNew.size(); i++) {
    if (!keepNew[i]) {
      dest << insChar << newList[i] << '\n';
      oldList.push_back (newList[i]);
    }
  }
}

std::vector<std::string> MessageBoardWriter::removeDuplicates (const std::vector<std::string>& list) {
  vector<string> dest;
  for (unsigned int i=0; i<list.size(); i++) {
    vector<string>::const_iterator it = find (dest.begin(), dest.end(), list[i]);
    if (it==dest.end()) {
      dest.push_back (list[i]);
    }
  }
  return dest;
}

void MessageBoardWriter::write (unsigned long int tav, const std::vector<std::string>& outgoing, const std::vector<std::string>& incoming) throw () {
  if (binary_mode || true) {  // so lange es noch keinen ASCII-Modus gibt, immer binaer
    wunsigned3(dest,tav);

    // die ausgehenden Mitteilungen abgleichen und verschicken
    matchMessages ('o', 'O', oldOutgoing, removeDuplicates (outgoing));
    // die eingehenden Mitteilungen abgleichen und verschicken
    matchMessages ('i', 'I', oldIncoming, removeDuplicates (incoming));
    dest << '%'; // naechster Zyklus
  }
}


MessageBoardReader::MessageBoardReader (std::istream& s) throw () : src(s), next(0), encoding_type(0) {
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
  if (encoding_type>=1 || true) {
    next = runsigned3(src);
  }
}

bool MessageBoardReader::next_timestamp (unsigned long int& t) const throw () {
  if (src.eof())
    return false;
  t=next;
  return true;
}

bool MessageBoardReader::read_until (unsigned long int& tav, std::vector<std::string>& outgoing, std::vector<std::string>& incoming, unsigned long int tuntil) throw (std::bad_alloc) {
  tav=next;
  bool read_anything = false;
  while (!src.eof() && next<=tuntil) {
    tav=next;
    if (encoding_type>=1 || true) {
      // binaere Codierung
      vector<bool> keepOutgoing (oldOutgoing.size(), true);
      vector<bool> keepIncoming (oldIncoming.size(), true);
      char op=' ';
      unsigned int idx;
      string line;
      while (!src.eof() && op!='%') {
        src.get (op);
        switch (op) {
          case '%': break;
          case 'o': idx = runsigned1 (src);
            if (idx<keepOutgoing.size()) keepOutgoing[idx]=false;
            break;
          case 'i': idx = runsigned1 (src);
            if (idx<keepIncoming.size()) keepIncoming[idx]=false;
            break;
          case 'O': getline (src, line);
            oldOutgoing.push_back (line);
            break;
          case 'I': getline (src, line);
            oldIncoming.push_back (line);
            break;
          default:
            break; // sollte nicht passieren
        }
      }
      for (unsigned int i=keepOutgoing.size(); i!=0; i--) {
        if (!keepOutgoing[i-1]) {
          oldOutgoing.erase (oldOutgoing.begin()+i-1);
        }
      }
      for (unsigned int i=keepIncoming.size(); i!=0; i--) {
        if (!keepIncoming[i-1]) {
          oldIncoming.erase (oldIncoming.begin()+i-1);
        }
      }

      next = runsigned3(src);
      read_anything=true;
    }
  }
  outgoing=oldOutgoing;
  incoming=oldIncoming;
  return read_anything;
}



#if 0

// Testmain:

#include <fstream>

int main () {
try{
  cout << "Nachrichten zum Verschicken eingeben. '%' markiert einen neuen Zyklus\n";
  cout << "Zum Beenden das Zeichen '#' eingeben\n";
  cout << "Ausgehende Zeilen beginnen mit 'O', eingehende mit 'I'\n";
  cout << "MessageBoard wird in die Datei 'msgb.log' geschrieben\n";

  {
    ofstream dest ("msgb.log");
    MessageBoardWriter writer (dest);

    bool quit=false;
    std::string line;
    vector<string> outgoing;
    vector<string> incoming;
    unsigned long int tt=0;
    while (!quit) {
      cout << "> ";
      cin >> line;
      if (line=="%" || line=="#") {
        writer.write (tt, outgoing, incoming);
        tt++;
        outgoing.clear();
        incoming.clear();
      } else if (line[0]=='O') {
        outgoing.push_back (line.substr(1,line.length()-1));
      } else if (line[0]=='I') {
        incoming.push_back (line.substr(1,line.length()-1));
      } else {
        cerr << "Fehlerhafte Zeile " << line << endl;
      }
      if (line=="#") {
        quit=true;
      }
    }
    dest << flush;
  }

  {
    cout << endl << "Lese aus Datei:\n";
    ifstream src ("msgb.log");
    MessageBoardReader reader (src);

    vector<string> outgoing;
    vector<string> incoming;
    bool success=true;
    unsigned long int tt;
    while (success) {
      success &= reader.next_timestamp (tt);
      success &= reader.read_until (tt, outgoing, incoming, tt);
      cout << "Ausgehend:\n";
      for (unsigned int i=0; i<outgoing.size(); i++)
        cout << outgoing[i] << endl;
      cout << "Eingehend:\n";
      for (unsigned int i=0; i<incoming.size(); i++)
        cout << incoming[i] << endl;
    }
  }
}catch(exception& e){ cerr << e.what() << endl; return -1; }
  return 0;
}

#endif
