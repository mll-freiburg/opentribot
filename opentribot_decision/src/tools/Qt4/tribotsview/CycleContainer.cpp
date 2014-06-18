
#include "CycleContainer.h"
#include "../../../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace TribotsTools;
using namespace std;

namespace {

  inline bool is_cycle_line (const char* s1) {
    if (s1[0]!='%' || s1[1]!='%' || s1[2]!='%' || s1[3]!='%' || s1[4]!='c' || s1[5]!='y' || s1[6]!='c' || s1[7]!='l' || s1[8]!='e' || s1[9]!=' ' ) return false;
    return true;
  }

  string next_word (const string& str, unsigned int& endpos, unsigned int firstpos) {
    if (str.length()<=firstpos) {
      return string("");
    } else {
      endpos = firstpos+1;
      bool was_whitespace = (isspace (str [firstpos]));
      while (endpos<str.length()) {
        if (isspace(str [endpos])) {
          if (!was_whitespace) {
            endpos--;
            return str.substr(firstpos,endpos-firstpos+1);
          }
        } else {
          was_whitespace=false;
        }
        endpos++;
      }
      // parser has reached end of string without whitespace
      endpos--;
      return (str.substr(firstpos,endpos-firstpos+1));
    }
  }

}



bool CycleContainer::read_next (CycleInfo& dest, unsigned long int n) throw () {
  if (!log_stream->good()) {
    cerr << "Warnung: Ende der Logdatei " << fname_praefix << ".log erreicht\n";    
    return false;
  }
  // .log-Datei einlesen
  std::string line;
  dest.time_msec = next_log_timestamp;
  dest.cycle_num = n;
  while (true) {
    getline (*log_stream, line);
    if (is_cycle_line (line.c_str()) || log_stream->eof()) {
      char* end_char;
      unsigned int p=10;
      string tw = next_word (line, p, p);
      next_log_timestamp = strtoul (tw.c_str(), &end_char, 0);
      break;
    } else {
      if (line.length()>0 && line[0]=='%')
        dest.paintmsg+=line.substr(1,line.length()-1)+string(" ");
      else if (line.substr(0,16)=="StartingTimeval ") {
        vector<string> parts;
        split_string (parts, line);
        if (parts.size()>=3) {
          string2lint (reftime.tv_sec, parts[1]);
          long int imli;
          string2lint (imli, parts[2]);
          reftime.tv_usec = imli;
        }
      } else
        dest.logmsg+=line+string("\n");
    }
  }
  if (log_stream->eof())
    return false;

  // Roboterposition lesen
  if (rloc_reader) {
    RobotLocation rl1, rl2;
    unsigned long int t0, t1, t2;
    if (rloc_reader->read_until (t0, t1, rl1, t2, rl2, next_log_timestamp+1)) {
      dest.rloc_vis.clear();
      dest.rloc_exec.clear();
      dest.rloc_vis.push_back (rl1);
      dest.rloc_exec.push_back (rl2);
    }else if (cycle_queue.size()>0) {
      dest.rloc_vis = cycle_queue.back().rloc_vis;
      dest.rloc_exec = cycle_queue.back().rloc_exec;
    }
  }
  // Ballposition einlesen
  if (bloc_reader) {
    BallLocation bl1, bl2;
    unsigned long int t0, t1, t2;
    if (bloc_reader->read_until (t0, t1, bl1, t2, bl2, next_log_timestamp+1)) {
      dest.bloc_vis.clear();
      dest.bloc_exec.clear();
      dest.bloc_vis.push_back (bl1);
      dest.bloc_exec.push_back (bl2);
    }
  }
  // DriveVector einlesen
  if (dv_reader) {
    DriveVector dv1;
     unsigned long int t0, t1;
    if (dv_reader->read_until (t0, t1, dv1, next_log_timestamp+1)) {
      dest.dv.clear();
      dest.dv.push_back (dv1);
    }
  }
  // Hindernisse einlesen
  if (oloc_reader) {
    unsigned long int t0;
    dest.oloc.pos.clear();
    dest.oloc.width.clear();
    oloc_reader->read_until (t0, dest.oloc, next_log_timestamp+1);
  }
  // Bildverarbeitungsinformation einlesen
  if (vloc_reader) {
    unsigned long int t0, t1;
    dest.vloc.clear();
    vloc_reader->read_until (t0, t1, dest.vloc, next_log_timestamp-1);
  }
  // Gamestate einlesen
  if (gs_reader) {
    unsigned long int t0;
    if (!gs_reader->read_until (t0, dest.gs, next_log_timestamp+1))
      dest.gs.refstate = Tribots::errorState;
  } else {
    dest.gs.refstate = Tribots::errorState;
  }
  // Partikel/Suchpositionen einlesen
  bool clean_ppos = true;
  while (ppos_stream && next_ppos_timestamp<next_log_timestamp+1 && !ppos_stream->eof()) {
    if (clean_ppos)
      dest.ppos.clear();
    unsigned int tlast=next_ppos_timestamp;
    string line;
    vector<string> parts;
    getline (*ppos_stream, line);
    (*ppos_stream) >> next_ppos_timestamp;
    split_string (parts, line);
    if (parts.size()>=4) {
      double trans;
      PossiblePosition pp;
      string2double (pp.pos.x, parts[0]);
      string2double (pp.pos.y, parts[1]);
      string2double (trans, parts[2]);
      pp.heading.set_rad (trans);
      string2double (pp.value, parts[3]);
      dest.ppos.push_back (pp);
      clean_ppos = (tlast!=next_ppos_timestamp);
    }
  }

  return true;
}

CycleContainer::CycleContainer (const char* praefix) throw (std::invalid_argument) {
  rloc_stream = bloc_stream = oloc_stream = vloc_stream = log_stream = ppos_stream = gs_stream = dv_stream = NULL;
  rloc_reader = NULL;
  bloc_reader = NULL;
  oloc_reader = NULL;
  vloc_reader = NULL;
  gs_reader = NULL;
  dv_reader = NULL;
  fname_praefix = praefix;
  if (!open_files (praefix))
    throw std::invalid_argument ("Fehler: Logdateien nicht vorhanden oder leer");
}

long int CycleContainer::step (long int d) throw () {
  unsigned long int req_cycle;
  if (d<0 && static_cast<unsigned long int>(-d)>=current_cycle)
    req_cycle=1;
  else
    req_cycle=current_cycle+d;

  return set (req_cycle);
}

long int CycleContainer::set (unsigned long int rc) throw () {
  if (rc==0)
    rc=1;
  bool okay=true;
  while (okay && cycle_queue.size()<rc) {  // ggf nachladen
    CycleInfo ci;
    okay &= read_next (ci, static_cast<unsigned long int>(cycle_queue.size()+1));
    if (okay) 
      cycle_queue.push_back (ci);
  }
  if (cycle_queue.size()==0)
    current_cycle=0;
  else if (cycle_queue.size()<rc)
    current_cycle = cycle_queue.size();
  else
    current_cycle = rc;
  return current_cycle;
}

long int CycleContainer::set_time (unsigned long int tt) throw () {
  bool okay=true;
  while (okay && cycle_queue[cycle_queue.size()-1].time_msec < tt) {  // ggf nachladen
    CycleInfo ci;
    okay &= read_next (ci, static_cast<unsigned long int>(cycle_queue.size()+1));
    if (okay) 
      cycle_queue.push_back (ci);
  }
  if (cycle_queue.size()==0)
    current_cycle=0;
  else {
    current_cycle=cycle_queue.size();
    while (current_cycle>1 && cycle_queue[current_cycle-1].time_msec > tt)
      current_cycle--;
  }
  return current_cycle;
}

const CycleInfo& CycleContainer::get () const throw () {
  return cycle_queue [current_cycle-1];
}

unsigned int CycleContainer::size () const throw () {
  return cycle_queue.size();
}

void CycleContainer::close_files () {
  if (rloc_reader) delete rloc_reader;
  if (bloc_reader) delete bloc_reader;
  if (oloc_reader) delete oloc_reader;
  if (vloc_reader) delete vloc_reader;
  if (gs_reader) delete gs_reader;
  if (dv_reader) delete dv_reader;

  if (rloc_stream) delete rloc_stream;
  if (bloc_stream) delete bloc_stream;
  if (oloc_stream) delete oloc_stream;
  if (vloc_stream) delete vloc_stream;
  if (log_stream) delete log_stream;
  if (ppos_stream) delete ppos_stream;
  if (gs_stream) delete gs_stream;
  if (dv_stream) delete dv_stream;
  rloc_stream = bloc_stream = oloc_stream = vloc_stream = log_stream = ppos_stream = gs_stream = dv_stream = NULL;
  rloc_reader = NULL;
  bloc_reader = NULL;
  oloc_reader = NULL;
  vloc_reader = NULL;
  gs_reader = NULL;
  dv_reader = NULL;
}

bool CycleContainer::open_files (const char* praefix) {
  rloc_stream = new ifstream ((string(praefix)+string(".rpos")).c_str());
  bloc_stream = new ifstream ((string(praefix)+string(".bpos")).c_str());
  oloc_stream = new ifstream ((string(praefix)+string(".opos")).c_str());
  vloc_stream = new ifstream ((string(praefix)+string(".vis")).c_str());
  log_stream = new ifstream ((string(praefix)+string(".log")).c_str());
  ppos_stream = new ifstream ((string(praefix)+string(".ppos")).c_str());
  gs_stream = new ifstream ((string(praefix)+string(".gs")).c_str());
  dv_stream = new ifstream ((string(praefix)+string(".drv")).c_str());

  if (!(*rloc_stream)) {  // falls keine RobotLocation-Datei existiert
    delete rloc_stream;
    rloc_stream=NULL;
    cerr << "Warnung: Datei " << praefix << ".rpos nicht gefunden\n";
  } else {
    rloc_reader = new RobotLocationReader (*rloc_stream);
  }

  if (!(*bloc_stream)) {  // falls keine BallLocation-Datei existiert
    delete bloc_stream;
    bloc_stream=NULL;
    cerr << "Warnung: Datei " << praefix << ".bpos nicht gefunden\n";
  } else {
    bloc_reader = new BallLocationReader (*bloc_stream);
  }

  if (!(*oloc_stream)) {  // falls keine ObstacleLocation-Datei existiert
    delete oloc_stream;
    oloc_stream=NULL;
    cerr << "Warnung: Datei " << praefix << ".opos nicht gefunden\n";
  } else {
    oloc_reader = new ObstacleLocationReader (*oloc_stream);
  }

  if (!(*vloc_stream)) {  // falls keine VisbleObject-Datei existiert
    delete vloc_stream;
    vloc_stream=NULL;
    cerr << "Warnung: Datei " << praefix << ".vis nicht gefunden\n";
  } else {
    vloc_reader = new VisibleObjectReader (*vloc_stream);
  }

  if (!(*gs_stream)) {  // falls keine GameState-Datei existiert
    delete gs_stream;
    gs_stream=NULL;
    cerr << "Warnung: Datei " << praefix << ".gs nicht gefunden\n";
  } else {
    gs_reader = new GameStateReader (*gs_stream);
  }

  if (!(*dv_stream)) {  // falls keine DriveVector-Datei existiert
    delete dv_stream;
    dv_stream=NULL;
    cerr << "Warnung: Datei " << praefix << ".gs nicht gefunden\n";
  } else {
    dv_reader = new DriveVectorReader (*dv_stream);
  }

  if (!(*ppos_stream)) {  // falls kein Selbstlokalisierungstream existiert
    delete ppos_stream;
    ppos_stream = NULL;
//    cerr << "Warnung: Datei " << praefix << ".ppos nicht gefunden\n";
  } else {
    (*ppos_stream) >> next_ppos_timestamp;
  }

  {
    synch_signals.clear ();
    ifstream src_syc ((string(praefix)+string(".syc")).c_str());
    if (src_syc) {
      while (!src_syc.eof()) {
        string line;
        getline (src_syc, line);
        vector<string> parts;
        split_string (parts, line);
        if (parts.size()>=2) {
          unsigned long int ts;
          unsigned int sig;
          string2ulint (ts, parts[0]);
          string2uint (sig, parts[1]);
          synch_signals.push_back (pair<unsigned long int, unsigned int>(ts, sig));
        }
      }
    }
  }

  if (*log_stream) {
    next_log_timestamp=0;
    CycleInfo ci;
    if (read_next (ci, 1)) {
      cycle_queue.clear();
      cycle_queue.push_back (ci);
      current_cycle=1;
      return true;
    }
  }
  return false;
}


bool CycleContainer::revert (const char* praefix) {
  fname_praefix=praefix;
  close_files ();
  return open_files (praefix);
}

bool CycleContainer::revert () {
  return revert (fname_praefix.c_str());
}

long int CycleContainer::cycle_num () const throw () {
  return current_cycle;
}

void  CycleContainer::replace (const CycleInfo& ci) {
  cycle_queue [current_cycle-1] = ci;
}

timeval CycleContainer::get_reference_time () const throw () {
  return reftime;
}

void CycleContainer::set_reference_time (const timeval& tv) throw () {
  reftime = tv;
}

long int CycleContainer::shift_time (long int tm, timeval ext_ref) const throw () {
  long int ds = ext_ref.tv_sec-reftime.tv_sec;
  long int dus = ext_ref.tv_usec-reftime.tv_usec;
  long int dms = 1000*ds+dus/1000;
  return dms+tm;
}

const std::deque<pair<unsigned long int, unsigned int> >& CycleContainer::get_synchronisation_signals () throw () {
  return synch_signals;
}
