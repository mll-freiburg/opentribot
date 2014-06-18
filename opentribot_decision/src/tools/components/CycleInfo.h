
#ifndef TribotsTools_CycleInfo_h
#define TribotsTools_CycleInfo_h

#include <string>
#include <vector>
#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/ObstacleLocation.h"
#include "../../Structures/VisibleObject.h"
#include "../../Structures/GameState.h"
#include "../../Structures/DriveVector.h"

namespace TribotsTools {

  /** Struktur, um Suchpositionen (z.B. Partikel) der Selbstlokalisation einzutragen */
  struct PossiblePosition {
    Tribots::Vec pos;         ///< x,y-Koordinaten
    Tribots::Angle heading;   ///< Orientierung
    double value;             ///< Guetewert
  };


  /** Struktur, um die Informationen eines Zykluses zu speichern */
  struct CycleInfo {
    unsigned long int cycle_num;                ///< Iteration
    unsigned long int time_msec;                ///< Programmzeit in msec
    unsigned int cycle_time;                    ///< Zykluszeit in msec

    std::vector<Tribots::RobotLocation> rloc_vis;   ///< Roboterdaten, ggf. fuer mehrere Roboter, zum Zeitpunkt der Bildinformation
    std::vector<std::vector<int> > robot_attr;      ///< Zusatzattribute fuer Roboter
    std::vector<Tribots::BallLocation> bloc_vis;    ///< Balldaten, ggf. fuer mehrere Roboter, zum Zeitpunkt der Bildinformation
    std::vector<Tribots::RobotLocation> rloc_exec;  ///< Roboterdaten, ggf. fuer mehrere Roboter, zum Zeitpunkt der Befehlsausfuehrung
    std::vector<Tribots::BallLocation> bloc_exec;   ///< Balldaten, ggf. fuer mehrere Roboter, zum Zeitpunkt der Befehlsausfuehrung
    Tribots::ObstacleLocation oloc;                 ///< Hindenisdaten
    std::vector<Tribots::VisibleObjectList> vloc;   ///< Gesehene Objekte
    std::vector<Tribots::DriveVector> dv;           ///< Fahrtvektoren, ggf. fuer mehrere Roboter, zum Zeitpunkt der Befehlsausfuehrung
    Tribots::GameState gs;                      ///< gegenwaertiger GameState
    std::string playertype;                     ///< Spielertyp
    std::string playerrole;                     ///< Spielerrolle
    std::string behavior;                       ///< Behavior
    std::string logmsg;                         ///< Loginfo
    std::string paintmsg;                       ///< Anweisungen, Linien zu zeichnen
    std::vector<PossiblePosition> ppos;         ///< Partikel des Condensation Filters o. ae.
    std::vector<std::string> mboard_outgoing;   ///< ausgehende Nachrichten des Messageboards
    std::vector<std::string> mboard_incoming;   ///< eingehende Nachrichten des Messageboards

    CycleInfo ();
    CycleInfo (const CycleInfo&);
    const CycleInfo& operator= (const CycleInfo&);
  };

}

#endif

