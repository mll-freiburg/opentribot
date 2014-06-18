
#include "BehaviorStatistics.h"
#include "../../Fundamental/stringconvert.h"
#include "../../Structures/GameStateReadWriter.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

namespace {

  struct CountStruct {
    std::string intention;
    unsigned long int count;
    bool operator< (const CountStruct& cs) const {
      return count>cs.count;  // umgekehrtes Zeichen richtig, da absteigend sortiert werden soll
    }
  };

  void increment_cs (vector<CountStruct>& cs, std::string key, int ct = 1) {
    // erhoeht den Eintrag von key um ct oder legt den Eintrag an
    for (unsigned int i=0; i<cs.size(); i++) {
      if (cs[i].intention==key) {
        cs[i].count+=ct;
        return;
      }
    }
    CountStruct css;
    css.intention=key;
    css.count=ct;
    cs.push_back (css);
  }
  
  void write_statistics (ostream& dest, const std::vector<CountStruct>& freq) throw () {
    dest.setf(ios_base::fixed,ios_base::floatfield);
    dest << setprecision(2);

    unsigned long int num_all = 0;
    unsigned long int num_so_far = 0;
    vector<CountStruct>::const_iterator itcs = freq.begin();
    vector<CountStruct>::const_iterator itcsend = freq.end();
    while (itcs<itcsend) {
      num_all+=itcs->count;
      itcs++;
    }
    itcs = freq.begin();
    unsigned int linenum=1;
    while (itcs<itcsend) {
      num_so_far += itcs->count;
      dest << '[' << linenum << ']' << '\t';
      dest.width(5);
      dest << itcs->count << ' ';
      dest.width(6);
      dest << static_cast<double>(itcs->count)/static_cast<double>(num_all)*100 << '%' << ' ';
      dest.width(5);
      dest << num_so_far << ' ';
      dest.width(6);
      dest << static_cast<double>(num_so_far)/static_cast<double>(num_all)*100 << '%' << ' ';
      dest << itcs->intention <<'\n';
      itcs++;
      linenum++;
    }
  }  
      
}


bool BehaviorStatistics::read_logfile (const char* filename) throw () {
  if (!read_gslog (string(filename)+".gs")) {
    std::cerr << "GameState-Logfile " << filename << ".gs nicht lesbar\n";
    refstates.assign (intentions.size(), errorState);
  }
  if (intentions.size()==0)
    read_textlog (string(filename)+".log");
  unstripped_intentions=intentions;
  
  while (refstates.size()<intentions.size()) {
    refstates.push_back (errorState);
  }
  calculate_next_standard ();
  return (intentions.size()>0);
}

bool BehaviorStatistics::read_gslog (const std::string& filename) throw () {
  ifstream src (filename.c_str());
  if (!src) return false;

  Tribots::GameStateReader reader (src);
  Tribots::GameState gs;
  std::string pt;
  std::string pr;
  std::string bh;

  unsigned long int next, t;
  bool nextex = reader.next_timestamp (next);
  while (nextex) {
	  //Arg1: Zeitstempel (Rueckgabe)
    //Arg2: letzter gelesener GameState (Rueckgabe)
    //Arg3: Spielertyp (Rueckgabe)
    //Arg4: Spielerrolle (Rueckgabe)
    //Arg5: Behavior (Rueckgabe)
    //Arg6: Zeitpunkt in ms, bis zu dem Objekte gelesen werden sollen
    reader.read_until (t, gs, pt, pr, bh, next);
    nextex = reader.next_timestamp (next);
    refstates.push_back (gs.refstate);
    if (bh.length()!=0)
      intentions.push_back (bh);
  }
  return true;
}

bool BehaviorStatistics::read_textlog (const std::string& filename) throw () {
  ifstream src (filename.c_str());
  if (!src) return false;
  
  string cintention = "";
  bool first=true;
  while (!src.eof()) {
    string line;
    getline (src, line);
    if (line.substr(0, 9)=="%%%%cycle") {
      if (cintention=="") {
        if (!first)
          intentions.push_back (string("--"));
      } else
        intentions.push_back (cintention);
      cintention="";
      first=false;
    } else if (line.substr(0, 19)=="Active Intention in" || line.substr(0,15)=="Active Stage in") {
      if (cintention.length()>0)
        cintention+="::";
      vector<string> words;
      split_string (words, line);
      cintention+=words[words.size()-1];
    }
  }
  if (cintention.length()>0)
    intentions.push_back (cintention);
  return true;
}


void BehaviorStatistics::write_intention_frequencies (ostream& dest) throw (std::bad_alloc) {
  vector<CountStruct> freq (0);
  vector<string> int2 = intentions;
  sort (int2.begin(), int2.end());

  string old_intention="";
  unsigned long int count =0;
  vector<string>::iterator it = int2.begin();
  vector<string>::iterator itend = int2.end();
  while (it<itend) {
    if (old_intention==(*it))
      count++;
    else if (old_intention=="") {
      count=1;
      old_intention=(*it);
    } else {
      CountStruct cs;
      cs.intention = old_intention;
      cs.count = count;
      freq.push_back (cs);
      count=1;
      old_intention=(*it);
    }
    it++;
  }
  if (old_intention!="") {
    CountStruct cs;
    cs.intention = old_intention;
    cs.count = count;
    freq.push_back (cs);
  }
  
  sort (freq.begin(), freq.end());
  
  write_statistics (dest, freq);
}

void BehaviorStatistics::write_episode_frequencies (ostream& dest) throw (std::bad_alloc) {
  vector<CountStruct> freq (0);
  
  string old_intention="";
  vector<string>::iterator it = intentions.begin();
  vector<string>::iterator itend = intentions.end();
  while (it!=itend) {
    if (old_intention!=(*it)) {
      old_intention=(*it);
      vector<CountStruct>::iterator itcs=freq.begin(); 
      for (; itcs!=freq.end(); itcs++) {
        if (itcs->intention==old_intention) {
          itcs->count++;
          break;
        }
      }
      if (itcs==freq.end()) {
        CountStruct csn;
        csn.intention=old_intention;
        csn.count=1;
        freq.push_back(csn);
      }
    }
    it++;
  }

  sort (freq.begin(), freq.end());
  
  write_statistics (dest, freq);
}


void BehaviorStatistics::write_successor_frequencies (ostream& dest, const char* beh) throw (std::bad_alloc) {
  string ref = beh;
  
  vector<CountStruct> freq (0);
  vector<string> int2 (0);
  string old = "";
  vector<string>::iterator it = intentions.begin();
  vector<string>::iterator itend = intentions.end();
  while (it<itend) {
    if (old==ref)
      int2.push_back (*it);
    old= *it;
    it++;
  }
  sort (int2.begin(), int2.end());
  
  string old_intention="";
  unsigned long int count =0;
  it = int2.begin();
  itend = int2.end();
  while (it<itend) {
    if (old_intention==(*it))
      count++;
    else if (old_intention=="") {
      count=1;
      old_intention=(*it);
    } else {
      CountStruct cs;
      cs.intention = old_intention;
      cs.count = count;
      freq.push_back (cs);
      count=1;
      old_intention=(*it);
    }
    it++;
  }
  if (old_intention!="") {
    CountStruct cs;
    cs.intention = old_intention;
    cs.count = count;
    freq.push_back (cs);
  }
  
  sort (freq.begin(), freq.end());
  
  write_statistics (dest, freq);
}


void BehaviorStatistics::write_predecessor_frequencies (ostream& dest, const char* beh) throw (std::bad_alloc) {
  string ref = beh;
  
  vector<CountStruct> freq (0);
  vector<string> int2 (0);
  string old = "";
  vector<string>::iterator it = intentions.end();  // passt schon, weil rueckwaerts durchlaufen
  vector<string>::iterator itend = intentions.begin();
  while (it>itend) {
    it--;
    if (old==ref)
      int2.push_back (*it);
    old= *it;
  }
  sort (int2.begin(), int2.end());
  
  string old_intention="";
  unsigned long int count =0;
  it = int2.begin();
  itend = int2.end();
  while (it<itend) {
    if (old_intention==(*it))
      count++;
    else if (old_intention=="") {
      count=1;
      old_intention=(*it);
    } else {
      CountStruct cs;
      cs.intention = old_intention;
      cs.count = count;
      freq.push_back (cs);
      count=1;
      old_intention=(*it);
    }
    it++;
  }
  if (old_intention!="") {
    CountStruct cs;
    cs.intention = old_intention;
    cs.count = count;
    freq.push_back (cs);
  }
  
  sort (freq.begin(), freq.end());
  
  write_statistics (dest, freq);
}

void BehaviorStatistics::write_cycles (ostream& dest, const char* beh, unsigned int st, unsigned int ln) throw () {
  if (ln==0) {
    ln=intentions.size()+100;
  }
  std::vector<std::string> int2 = intentions;
  int2.push_back ("");
  std::vector<RefereeState> ref2 = refstates;
  ref2.push_back (errorState);
  bool special_behavior = (string(beh)!=string(""));

  std::string pred_intention="";
  std::string old_intention="";
  RefereeState old_refstate=errorState;
  unsigned int old_time_next_standard=0;
  RefereeState old_next_standard=errorState;
  unsigned long int cycle=0;
  unsigned long int start=0;
  vector<string>::iterator it = int2.begin()+st;
  vector<RefereeState>::iterator rsit = ref2.begin()+st;
  vector<RefereeState>::iterator nsit = next_standard.begin()+st;
  vector<unsigned int>::iterator tnsit = time_next_standard.begin()+st;
  while (it<int2.end() && it<int2.begin()+st+ln+1) {
    cycle++;
    if (((*it)!=old_intention) || ((*rsit)!=old_refstate) || (it==int2.begin()+st+ln)) {
      if ((special_behavior && old_intention==beh) || (!special_behavior && old_intention!="")) {
        dest.width(5);
        if (start+1==cycle) {
          dest << start+st << "        ";
        } else {
          dest << start+st << " - ";
          dest.width(5);
          dest << st+cycle-1;
        }
        if (special_behavior) {
          dest << ' ' << referee_state_names[old_refstate] << ' ' << pred_intention << " - " << (*it) << " -> " << old_time_next_standard << ',' << referee_state_names[old_next_standard];
        } else {
          dest << ' ' << referee_state_names[old_refstate] << ' ' << old_intention;
        }
        dest << '\n';
      }
      if (it+1==int2.end())
        return;
      start=cycle;
      pred_intention=old_intention;
      old_intention=(*it);
      old_refstate=(*rsit);
      old_next_standard=(*nsit);
      old_time_next_standard=(*tnsit);
    }
    it++;
    rsit++;
    nsit++;
    tnsit++;
  }
}

void BehaviorStatistics::strip_hierarchy (bool b) throw (std::bad_alloc) {
  if (b) {
    intentions=unstripped_intentions;
  } else {
    for (std::vector<std::string>::iterator it=intentions.begin(); it!=intentions.end(); it++) {
      unsigned int index = it->rfind ("::",it->length());
      if (index!=string::npos) {
        (*it)=it->substr (index+2, it->length());
      }
    }
  }
}

void BehaviorStatistics::transition_graph (const char* name, const char* bl2) throw (std::bad_alloc) {
  string bl (bl2);
  std::vector<std::string> behaviors;
  unsigned int kindex = 0;
  while (true) {
    unsigned int kindex2 = bl.find (",",kindex);
    if (kindex2!=string::npos) {
      behaviors.push_back (bl.substr (kindex, kindex2-kindex));
      kindex=kindex2+1;
    } else {
      if (kindex<bl.size()) {
        behaviors.push_back (bl.substr (kindex, bl.length()));
      }
      break;
    }
  }

  if (behaviors.size()==0) {
    behaviors=intentions;
  }
  std::sort (behaviors.begin(), behaviors.end());
  vector<string>::iterator p = std::unique (behaviors.begin(), behaviors.end());
  behaviors.erase (p, behaviors.end());

  const unsigned int num=behaviors.size();
  std::vector<unsigned int> connection (num*num);

  for (std::vector<unsigned int>::iterator it = connection.begin(); it!=connection.end(); it++) {
    (*it)=0;
  }

  // Statistik erstellen
  std::string old_behavior="";
  bool old_valid=false;
  unsigned int old_index=0;
  for (std::vector<std::string>::iterator it = intentions.begin(); it!=intentions.end(); it++) {
    std::vector<std::string>::iterator itb = lower_bound (behaviors.begin(), behaviors.end(), *it);
    if (itb!=behaviors.end() && (*itb)==(*it)) {
      unsigned int new_index=(itb-behaviors.begin());
      if (old_valid) {
        (connection[old_index*num+new_index])++;
      }
      old_valid=true;
      old_index=new_index;
    } else {
      old_valid=false;
    }

    old_behavior = (*it);
  }

  // pstricks-Grafik erstellen
  string texname = string(name)+".tex";
  string dviname = string(name)+".dvi";
  string pdfname = string(name)+".pdf";
  ofstream ltx (texname.c_str());
  if (!ltx) {
    std::cerr << "Latex-Datei " << texname << " kann nicht erstellt werden.\n";
    return;
  }

  double picsz=(num>10 ? 1.5*num : 15);
  double picszx=119; //picsz+30;
  double picszy=84; //picsz+2;
  ltx << "\\documentclass{a0poster}\n";
  ltx << "\\usepackage{pstricks}\n";
  ltx << "\\usepackage{pst-node}\n";
  ltx << "\\begin{document}\n";
  ltx << "\\psset{xunit=1cm,yunit=1cm,runit=1cm}\n";
  ltx << "\\psset{nodesep=3pt,nrot=:U,arrowsize=5pt}\n";
  ltx << "\\begin{minipage}[b][" << picszy << "cm][b]{" << picszx << "cm}\n";
  ltx << "\\begin{pspicture}(" << picszx << ',' << picszy << ")\n";
  for (unsigned int i=0; i<num; i++) {
    double posx = 0.5*picszx+(0.5*picsz-0.4)*cos(2*M_PI*static_cast<double>(i)/static_cast<double>(behaviors.size()));
    double posy = 0.5*picszy+(0.5*picsz-0.4)*sin(2*M_PI*static_cast<double>(i)/static_cast<double>(behaviors.size()));
    double posxt = 0.5*picszx+(0.5*picsz)*cos(2*M_PI*static_cast<double>(i)/static_cast<double>(behaviors.size()));
    double posyt = 0.5*picszy+(0.5*picsz)*sin(2*M_PI*static_cast<double>(i)/static_cast<double>(behaviors.size()));
    char orientx = (posx>0.5*picszx ? 'l' : 'r');
    char orienty = (posy>0.5*picszy ? 'b' : 't');
    ltx << "\\rput[" << orientx << orienty << "](" << posxt << ',' << posyt << "){\\mbox{" << behaviors[i] << "}}\n";
    ltx << "\\cnode(" << posx << ',' << posy << "){0.2cm}{N" << i << "}\n";
  }

  for (unsigned int i=0; i<num; i++) {
    for (unsigned int j=0; j<num; j++) {
      if (i!=j) {
        unsigned int n=connection[i*num+j];
        if (n>0) {
          unsigned int wdth=(n>22 ? 6 : (n>15 ? 5 : (n>10 ? 4 : (n>7 ? 3 : (n>4 ? 2 : 1)))));
          unsigned int arrowwdth = (3*wdth>7 ? 3*wdth : 7);
          ltx << "\\ncarc[linewidth=" << wdth << "pt,arrowsize=" << arrowwdth << "pt]{->}{N" << i << "}{N" << j << "}\n"; //\\naput{" << n << "}\n";
        }
      }
    }
  }
  ltx << "\\end{pspicture}\n";
  ltx << "\\end{minipage}\n";
  ltx << "\\end{document}\n";
  ltx << flush;

  bool nonsuccess = system ((string("latex ")+texname+" >/tmp/latex.log").c_str());
  nonsuccess |= system ((string("dvipdf ")+dviname+" "+pdfname+" 2>/tmp/dvipdf.log").c_str());
  if (nonsuccess) {
    cout << "generated latex-file \"" << texname << "\" but was unable to generate ps file\n";
  } else {
    cout << "generated transition graph in file \"" << pdfname << "\"\n";
    nonsuccess |= system ((string("nohup evince ")+pdfname+" 2>/tmp/nohup.log &").c_str());
  }
}


void BehaviorStatistics::calculate_next_standard () throw (std::bad_alloc) {
  next_standard = refstates;
  time_next_standard.assign (next_standard.size(),0);
  RefereeState rs = errorState;
  unsigned int count=0;
  vector<RefereeState>::reverse_iterator rit = next_standard.rbegin();
  vector<unsigned int>::reverse_iterator cit = time_next_standard.rbegin();
  while (rit != next_standard.rend()) {
    if ((*rit)==preOwnKickOff || (*rit)==preOpponentKickOff || (*rit)==preOwnFreeKick || (*rit)==preOpponentFreeKick || (*rit)==preOwnThrowIn || (*rit)==preOpponentThrowIn || (*rit)==preOwnCornerKick || (*rit)==preOpponentCornerKick || (*rit)==preOwnGoalKick || (*rit)==preOpponentGoalKick || (*rit)==preOwnPenalty || (*rit)==preOpponentPenalty) {
      rs=(*rit);
      count=0;
    }
    (*cit)=count;
    if ((*rit)!=stopRobot)
      count++;
    (*rit)=rs;
    rit++;
    cit++;
  }
}

void BehaviorStatistics::next_standard_frequencies (std::ostream& os, const char* behavior, const unsigned int nmax) throw (std::bad_alloc) {
  vector<CountStruct> freq (0);
  vector<string> int2 = intentions;
  int2.push_back ("");
  std::string old_intention = "";
  Tribots::RefereeState old_refstate = errorState;
  unsigned int old_time = 0;
  vector<std::string>::iterator intit = int2.begin();
  vector<RefereeState>::iterator rsit = next_standard.begin();
  vector<unsigned int>::iterator cit = time_next_standard.begin();
  while (intit!=int2.end()) {
    if ((*intit)!=old_intention) {
      if (old_intention==behavior && (nmax==0 || old_time<=nmax)) {
        increment_cs (freq, referee_state_names [old_refstate]);
      }
      old_intention = (*intit);
    }
    old_refstate = (*rsit);
    old_time = (*cit);
    intit++;
    rsit++;
    cit++;
  }

  write_statistics (os, freq);
}
