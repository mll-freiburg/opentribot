
#ifndef _TribotsTools_CoachLogger_h_
#define _TribotsTools_CoachLogger_h_

#include <iostream>
#include <fstream>
#include "../../../Structures/RobotLocationReadWriter.h"
#include "../../../Structures/BallLocationReadWriter.h"
#include "../../../Structures/GameStateReadWriter.h"
#include "../../../Structures/MessageBoardReadWriter.h"

namespace TribotsTools {

  /** Logger fuers Teamcontrol, als Singleton */
  class CoachLogger {
  public:
    static CoachLogger& getCoachLogger () throw ();

    ~CoachLogger () throw ();

    bool create (const char* dir = ".") throw ();  ///< Logfiles erzeugen in Verzeichnis dir
    void destroy () throw ();  ///< Logfiles schliessen

    void log_cycleend () throw ();  ///< Loggen am Zyklusbegin
    void log_cyclebegin () throw ();  ///< Loggen am Ende des Zyklus

    std::ostream& log_stream () throw ();  ///< der Logstream
  private:
    CoachLogger () throw ();  // private, da Singleton
    static CoachLogger* the_coach_logger_pointer;

    bool active;
    unsigned int num_robots;
    unsigned long int cycle;

    std::ofstream** robotout;
    std::ofstream** ballout;
    std::ofstream** gamestateout;
    std::ofstream** messageboardout;
    std::ofstream* logout;
    std::ofstream* sycout;
    Tribots::RobotLocationWriter** robotwriter;
    Tribots::BallLocationWriter** ballwriter;
    Tribots::GameStateWriter** gamestatewriter;
    Tribots::MessageBoardWriter** messageboardwriter;

    std::ofstream devnull;
  };

  #define LOUT TribotsTools::CoachLogger::getCoachLogger().log_stream()

}

#endif
