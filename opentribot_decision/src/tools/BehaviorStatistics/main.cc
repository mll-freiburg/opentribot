
#include "BehaviorStatistics.h"
#include "../../Fundamental/stringconvert.h"
#include <iostream>
#include <deque>

using namespace std;

namespace {
  
  void write_helptext (ostream& dest, const char* progname) {
    dest << "Programm zum Auswerten der Intentionen aus einem Logfile\n";
    dest << "Aufruf: " << progname << " Logfile [Kommandos]\n";
    dest << "Folgende Kommandos sind implementiert:\n";
    dest << " all -- liefert eine Summenstatistik ueber alle Intentionen\n";
    dest << " epi -- liefert eine Summenstatistik der Episoden\n";
    dest << " pred X -- liefert eine Summenstatistik ueber alle Vorgaengerintentionen von Intention X\n";
    dest << " succ X -- liefert eine Summenstatistik ueber alle Nachfolgeintentionen von Intention X\n";
    dest << " list -- liefert alle Behaviors und GameStates in ihrer zeitlichen Reihenfolge\n";
    dest << " interval n1 n2 -- liefert alle Behaviors und GameStates in ihrer zeitlichen Reihenfolge beginnend von Zyklus n1 bis Zyklus n1+n2\n";
    dest << " cycles X -- liefert die Zyklen, in denen Behavior X aktiv war sowie GameState und Zeit bis zur naechsten Standardsituation\n";
    dest << " nextstandard X T -- liefert Summenstatistik ueber die nachste Standardsituation nach Behavior X maximal T Zyklen spaeter\n";
    dest << " graph X1,X2,X3,... -- erzeugt einen Uebergangsgraphen fuer die angegebenen Behaviors\n";
    dest << " allgraph -- erzeugt einen Uebergangsgraphen fuer alle Behaviors\n";
    dest << " strip -- Anzeige ohne Behavior-Hierarchie\n";
    dest << " unstrip -- Anzeige mit Behavior-Hierarchie\n";
    dest << " help -- dieser Text\n";
    dest << " quit, exit -- Programm beenden\n";
  }

}


int main (int argc, char** argv) {
  if (argc<2) {
    write_helptext (cerr, argv[0]);
    return -1;
  }
  try{
    deque<string> commands;
    for (int i=2; i<argc; i++)
      commands.push_back (string(argv[i]));
    
    TribotsTools::BehaviorStatistics stat;
    if (!stat.read_logfile (argv[1])) {
      cerr << "Logdatei nicht lesbar.\n";
      return -1;
    }
    
    cout.precision (4);
    
    bool need_more_arguments=false;
    while (true) {
      if (commands.size()==0 || need_more_arguments) {
        if (need_more_arguments) {
          cout << "  > ";
        } else {
          cout << "BehaviorStatistics> ";
        }
        string newcom;
        getline (cin, newcom);
        vector<string> addcom;
        Tribots::split_string (addcom, newcom);
        commands.insert (commands.end(), addcom.begin(), addcom.end());
        need_more_arguments=false;
      } else {
        need_more_arguments=false;
        if (commands[0]=="exit" || commands[0]=="quit")
          break;
        else if (commands[0]=="help") {
          write_helptext (cout, argv[0]);
          commands.pop_front();
        } else if (commands[0]=="all") {
          stat.write_intention_frequencies (cout);
          commands.pop_front();
        } else if (commands[0]=="epi") {
          stat.write_episode_frequencies (cout);
          commands.pop_front();
        } else if (commands[0]=="list") {
          stat.write_cycles (cout,"");
          commands.pop_front();
        } else if (commands[0]=="interval") {
          if (commands.size()>=3) {
            unsigned int n1=0, n2=0;
            Tribots::string2uint (n1, commands[1].c_str());
            Tribots::string2uint (n2, commands[2].c_str());
            stat.write_cycles (cout, "", n1, n2);
            commands.pop_front();
            commands.pop_front();
            commands.pop_front();
          } else
            need_more_arguments=true;
        } else if (commands[0]=="strip") {
          stat.strip_hierarchy (false);
          commands.pop_front();
        } else if (commands[0]=="unstrip") {
          stat.strip_hierarchy (true);
          commands.pop_front();
        } else if (commands[0]=="allgraph") {
          stat.transition_graph ("behavior_transition", "");
          commands.pop_front();
        } else if (commands[0]=="graph") {
          if (commands.size()>=2) {
            stat.transition_graph ("behavior_transition", commands[1].c_str());
            commands.pop_front();
            commands.pop_front();
          } else
            need_more_arguments=true;
        } else if (commands[0]=="pred") {
          if (commands.size()>=2) {
            stat.write_predecessor_frequencies (cout, commands[1].c_str());
            commands.pop_front();
            commands.pop_front();
          } else
            need_more_arguments=true;
        } else if (commands[0]=="succ") {
          if (commands.size()>=2) {
            stat.write_successor_frequencies (cout, commands[1].c_str());
            commands.pop_front();
            commands.pop_front();
          } else
            need_more_arguments=true;
        } else if (commands[0]=="cycles") {
          if (commands.size()>=2) {
            stat.write_cycles (cout, commands[1].c_str());
            commands.pop_front();
            commands.pop_front();
          } else
            need_more_arguments=true;
        } else if (commands[0]=="nextstandard") {
          if (commands.size()>=3) {
            unsigned int tt=0;
            Tribots::string2uint (tt, commands[2].c_str());
            stat.next_standard_frequencies (cout, commands[1].c_str(), tt);
            commands.pop_front();
            commands.pop_front();
            commands.pop_front();
          } else
            need_more_arguments=true;
        } else {
          cerr << "unbekanntes Kommando " << commands[0] << '\n';
          commands.pop_front();
        }
      }
    }
  }catch(exception& e){
    cerr << e.what() << endl;
    return -1;
  }
  return 0;
}
