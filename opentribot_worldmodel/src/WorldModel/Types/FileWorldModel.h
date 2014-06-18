
#ifndef _Tribots_FileWorldModel_h_
#define _Tribots_FileWorldModel_h_

#include "WorldModelTypeBase.h"
#include "../SL/RobotPositionPredictor.h"
#include "../../Structures/GameStateReadWriter.h"
#include "../../Structures/RobotLocationReadWriter.h"
#include "../../Structures/BallLocationReadWriter.h"
#include "../../Structures/ObstacleLocationReadWriter.h"
#include <fstream>

namespace Tribots {

  /** Klasse FileWorldModel liest die Roboter- und Ballpositionen aus
      Dateien ein, die mit AddWriteWorldModel erzeugt wurden */
  class FileWorldModel : public WorldModelTypeBase {
  private:
    std::ifstream* rpos_stream;              // einige Eingabestreams
    std::ifstream* bpos_stream;
    std::ifstream* opos_stream;
    std::ifstream* gs_stream;

    RobotLocationReader* robot_reader;       // einige Readerobjekte
    BallLocationReader* ball_reader;
    ObstacleLocationReader* obstacle_reader;
    GameStateReader* gs_reader;

    RobotPositionPredictor rpos_predict;     // Prognosetool fuer Position
    ObstacleLocation obstacles;              // Liste der Hindernisse
    BallLocation bloc;                       // letzte eingelesene Ballposition
    Time latest_bloc;                        // Zeitpunkt der letzten Ballinformation
    Time latest_vis_timestamp;

  public:
    /** Konstruktor, die Dateinamen werden aus ConfigReader gelesen */
    FileWorldModel (const ConfigReader&) throw (std::bad_alloc, Tribots::InvalidConfigurationException);
    ~FileWorldModel () throw ();

    RobotLocation get_robot (Time) const throw ();
    BallLocation get_ball (Time) const throw ();
    ObstacleLocation get_obstacles (Time) const throw ();
    Time get_timestamp_latest_update () const throw ();

    /** liest Informationen aus den Dateien ein */
    unsigned update_localisation () throw ();
    /** tut nichts */
    void reset () throw ();
    /** tut nichts */
    void reset (const Vec) throw ();
  };

}

#endif
