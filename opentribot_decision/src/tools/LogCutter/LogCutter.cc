
#include "../../Structures/BallLocationReadWriter.h"
#include "../../Structures/DriveVectorReadWriter.h"
#include "../../Structures/GameStateReadWriter.h"
#include "../../Structures/ObstacleLocationReadWriter.h"
#include "../../Structures/RobotLocationReadWriter.h"
#include "../../Structures/VisibleObjectReadWriter.h"
#include "../../Structures/MessageBoardReadWriter.h"
#include "../../Fundamental/stringconvert.h"
#include <fstream>

using namespace Tribots;
using namespace std;

int main (int argc, char** argv) {
  if (argc<4) {
    cerr << "Aufruf: " << argv[0] << " Zieldatei-Praefix Quelldatei-Praefix Zeitraum(s) [-a]\n";
    cerr << "schneidet einen gegebenen Zeitraum aus Logfiles aus\n";
    cerr << "Formatierung des Zeitraums: von-bis\n";
    cerr << "-a: in Klartext (ascii) schreiben (default:binaer)\n";
    cerr << "Beruecksichtigt Robot- Ball-, ObstacleLocation, Odometrie, Fahrtvektroen, Bildinformation, .log-Datei\n";
    return -1;
  }
  try{

    bool binary = true;
    string src_praefix = argv[2];
    string dest_praefix = argv[1];
    string interval = argv[3];
    if (argc>4 && argv[4][0]=='-' && argv[4][1]=='a')
      binary=false;
    unsigned int idash = interval.find ('-');
    if (idash>interval.length())
      throw invalid_argument ("Zeitraum nicht korrekt angegeben");
    unsigned long int from=0;
    unsigned long int to=999999999;
    if (idash>0) {
      double d;
      string2double (d, interval.substr(0,idash));
      from=static_cast<unsigned long int>(1000*d);
    }
    if (idash+1<interval.length()) {
      double d;
      string2double (d, interval.substr(idash+1,interval.length()));
      to=static_cast<unsigned long int>(1000*d);
    }

    // RobotLocation:
    {
      ifstream src ((src_praefix+".rpos").c_str());
      RobotLocationReader reader (src);
      ofstream dest ((dest_praefix+".rpos").c_str());
      RobotLocationWriter writer (dest,binary);
      unsigned long int tt;
      bool neof = reader.next_timestamp (tt);
      while (neof) {
        unsigned long int t1, t2;
        RobotLocation info1, info2;
        reader.read_until (tt, t1, info1, t2, info2, tt);
        if (from<=tt && tt<=to)
          writer.write (tt-from, t1-from, info1, t2-from, info2);
        neof = reader.next_timestamp (tt);
      }
      dest << flush;
    }

    // BallLocation:
    {
      ifstream src ((src_praefix+".bpos").c_str());
      BallLocationReader reader (src);
      ofstream dest ((dest_praefix+".bpos").c_str());
      BallLocationWriter writer (dest,binary);
      unsigned long int tt;
      bool neof = reader.next_timestamp (tt);
      while (neof) {
        unsigned long int t1, t2;
        BallLocation info1, info2;
        reader.read_until (tt, t1, info1, t2, info2, tt);
        if (from<=tt && tt<=to)
          writer.write (tt-from, t1-from, info1, t2-from, info2);
        neof = reader.next_timestamp (tt);
      }
      dest << flush;
    }

    // ObstacleLocation:
    {
      ifstream src ((src_praefix+".opos").c_str());
      ObstacleLocationReader reader (src);
      ofstream dest ((dest_praefix+".opos").c_str());
      ObstacleLocationWriter writer (dest, binary);
      unsigned long int tt;
      bool neof = reader.next_timestamp (tt);
      while (neof) {
        ObstacleLocation info1;
        reader.read_until (tt, info1, tt);
        if (from<=tt && tt<=to)
          writer.write (tt-from, info1);
        neof = reader.next_timestamp (tt);
      }
      dest << flush;
    }

    // DriveVector:
    {
      ifstream src ((src_praefix+".drv").c_str());
      DriveVectorReader reader (src);
      ofstream dest ((dest_praefix+".drv").c_str());
      DriveVectorWriter writer (dest,binary);
      unsigned long int tt;
      bool neof = reader.next_timestamp (tt);
      while (neof) {
        unsigned long int t1;
        DriveVector info1;
        reader.read_until (tt, t1, info1, tt);
        if (from<=tt && tt<=to)
          writer.write (tt-from, t1-from, info1);
        neof = reader.next_timestamp (tt);
      }
      dest << flush;
    }

    // Odometrie:
    {
      ifstream src ((src_praefix+".odo").c_str());
      DriveVectorReader reader (src);
      ofstream dest ((dest_praefix+".odo").c_str());
      DriveVectorWriter writer (dest,binary);
      unsigned long int tt;
      bool neof = reader.next_timestamp (tt);
      while (neof) {
        unsigned long int t1;
        DriveVector info1;
        reader.read_until (tt, t1, info1, tt);
        if (from<=tt && tt<=to)
          writer.write (tt-from, t1-from, info1);
        neof = reader.next_timestamp (tt);
      }
      dest << flush;
    }

    // MessageBoard:
    {
      ifstream src ((src_praefix+".mbd").c_str());
      ofstream dest ((dest_praefix+".mbd").c_str());
      if (src) {
        MessageBoardReader reader (src);
        MessageBoardWriter writer (dest,binary);
        unsigned long int tt;
        bool neof = reader.next_timestamp (tt);
        vector<string> incoming, outgoing;
        while (neof) {
          reader.read_until (tt, outgoing, incoming, tt);
          if (from<=tt && tt<=to)
            writer.write (tt-from, outgoing, incoming);
          neof = reader.next_timestamp (tt);
        }
        dest << flush;
      }
    }

    // VisibleObject:
    {
      ifstream src ((src_praefix+".vis").c_str());
      VisibleObjectReader reader (src);
      ofstream dest ((dest_praefix+".vis").c_str());
      VisibleObjectWriter writer (dest, binary);
      unsigned long int tt;
      bool neof = reader.next_timestamp (tt);
      while (neof) {
        unsigned long int t1;
        vector<VisibleObjectList> info1;
        reader.read_until (tt, t1, info1, tt);
        if (from<=tt && tt<=to)
          for (unsigned int i=0; i<info1.size(); i++)
            writer.write (tt-from, t1-from, info1[i], i);
        neof = reader.next_timestamp (tt);
      }
      dest << flush;
    }

    // GameState:
    {
      ifstream src ((src_praefix+".gs").c_str());
      if (src) {
        string playertype;
        string playerrole;
        string behavior;
        GameStateReader reader (src);
        ofstream dest ((dest_praefix+".gs").c_str());
        GameStateWriter writer (dest, binary);
        unsigned long int tt;
        bool neof = reader.next_timestamp (tt);
        while (neof) {
          GameState info1;
          reader.read_until (tt, info1, playertype, playerrole, behavior, tt);
          if (from<=tt && tt<=to)
            writer.write (tt-from, info1, playertype.c_str(), playerrole.c_str(), behavior.c_str());
          neof = reader.next_timestamp (tt);
        }
        dest << flush;
      }
    }

    // .log-Datei:
    {
      ifstream src ((src_praefix+".log").c_str());
      ofstream dest ((dest_praefix+".log").c_str());
      bool is_active=false;
      while (!src.eof()) {
        string line;
        getline (src, line);
        if (src.eof())
          break;
        if (prefix (string("\%\%\%\%cycle"), line)) {
          unsigned int idx = line.find ('\t');
          unsigned long int tt;
          string2ulint (tt, line.substr (10, idx-10));
          if (from<=tt && tt<=to) {
            dest << line.substr(0,10) << tt-from << line.substr (idx, line.length()) << '\n';
            is_active=true;
          } else
            is_active=false;
        } else if (prefix (string("FieldGeometry:"), line)) {
          dest << line << '\n';  // Feldgeometrie-Information auf jeden Fall uebernehmen
        } else if (prefix (string("StartingTimeval"), line)) {
          // StartingTimeval uebernehmen und anpassen
          vector<string> parts;
          split_string (parts, line);
          if (parts.size()>=3) {
            unsigned long int tv_sec, tv_usec;
            string2ulint (tv_sec, parts[1]);
            string2ulint (tv_usec, parts[2]);
            tv_sec += (from/1000);
            tv_usec += (from%1000)*1000;
            if (tv_usec>=1000000) {
              tv_sec++;
              tv_usec-=1000000;
            }
            dest << "StartingTimeval " << tv_sec << ' ' << tv_usec << '\n';
          }
        } else if (is_active) {
          dest << line << '\n';
        }
      }
      dest << flush;
    }

  }catch(std::exception& e){
    cerr << e.what() << endl;
    return -1;
  }
  return 0;
}
