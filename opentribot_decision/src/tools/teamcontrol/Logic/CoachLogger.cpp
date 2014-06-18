
#include "CoachLogger.h"
#include "../States/RemoteBlackboard.h"
#include <string>
#include <cstdlib>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

namespace {

  void make_link (const std::string& to, const std::string& from) {
    system ((string("rm -f ")+from+ " 2> /dev/null").c_str());
    system ((string("ln -s ")+to+" "+from+" 2> /dev/null").c_str());
  }

}


CoachLogger* CoachLogger::the_coach_logger_pointer = NULL;

CoachLogger& CoachLogger::getCoachLogger () throw () {
  if (!the_coach_logger_pointer)
    the_coach_logger_pointer = new CoachLogger;
  return *the_coach_logger_pointer;
}

CoachLogger::CoachLogger () throw () :
    active (false),
    num_robots(0),
    cycle (0),
    logout (NULL),
    sycout (NULL),
    devnull ("/dev/null") {;}

CoachLogger::~CoachLogger () throw () {
  if (active)
    destroy ();
}

void CoachLogger::destroy () throw () {
  if (!active) {
    num_robots=0;
    return;
  }
  for (unsigned int i=0; i<num_robots; i++) {
    if (robotwriter[i]) delete robotwriter[i];
    if (ballwriter[i]) delete ballwriter[i];
    if (gamestatewriter[i]) delete gamestatewriter[i];
    if (messageboardwriter[i]) delete messageboardwriter[i];
    if (robotout[i]) (*robotout[i]) << flush;
    if (ballout[i]) (*ballout[i]) << flush;
    if (gamestateout[i]) (*gamestateout[i]) << flush;
    if (messageboardout[i]) (*messageboardout[i]) << flush;
    if (robotout[i]) delete robotout[i];
    if (ballout[i]) delete ballout[i];
    if (gamestateout[i]) delete gamestateout[i];
    if (messageboardout[i]) delete messageboardout[i];
  }
  delete [] robotwriter;
  delete [] ballwriter;
  delete [] gamestatewriter;
  delete [] messageboardwriter;
  delete [] robotout;
  delete [] ballout;
  delete [] gamestateout;
  delete [] messageboardout;
  if (logout) (*logout) << flush;
  if (sycout) (*sycout) << flush;
  if (logout) delete logout;
  if (sycout) delete sycout;
  active=false;
  num_robots=0;
  logout=NULL;
  sycout=NULL;
}

bool CoachLogger::create (const char* dir) throw () {
  if (active)
    destroy ();

  system ((string("mkdir -p ")+dir+"/tclog").c_str());
  string pref = string(dir)+"/tclog/coachlog";
  bool success = true;
  num_robots=REMBB.robot_state.size();
  robotout = new ofstream* [num_robots];
  ballout = new ofstream* [num_robots];
  gamestateout = new ofstream* [num_robots];
  messageboardout = new ofstream* [num_robots];
  robotwriter = new RobotLocationWriter* [num_robots];
  ballwriter = new BallLocationWriter* [num_robots];
  gamestatewriter = new GameStateWriter* [num_robots];
  messageboardwriter = new MessageBoardWriter* [num_robots];
  for (unsigned int i=0; i<num_robots; i++) {
    string id = "00";
    id[0]='0'+REMBB.robot_state[i].id/10;
    id[1]='0'+REMBB.robot_state[i].id%10;
    robotout[i] = new ofstream ((pref+"_"+id+".rpos").c_str());
    ballout[i] = new ofstream ((pref+"_"+id+".bpos").c_str());
    gamestateout[i] = new ofstream ((pref+"_"+id+".gs").c_str());
    messageboardout[i] = new ofstream ((pref+"_"+id+".mbd").c_str());
    success &= (*robotout[i]) && (*ballout[i]) && (*gamestateout[i]) && (*messageboardout[i]);
    if (success) {
      robotwriter[i] = new RobotLocationWriter (*robotout[i]);
      ballwriter[i] = new BallLocationWriter (*ballout[i]);
      gamestatewriter[i] = new GameStateWriter (*gamestateout[i]);
      messageboardwriter[i] = new MessageBoardWriter (*messageboardout[i]);
    } else {
      robotwriter[i]=NULL;
      ballwriter[i]=NULL;
      gamestatewriter[i]=NULL;
      messageboardwriter[i]=NULL;
    }
    make_link ("coachlog.clog", pref+"_"+id+".log");
    make_link ("coachlog.csyc", pref+"_"+id+".syc");
  }
  logout = new ofstream ((pref+".clog").c_str());
  sycout = new ofstream ((pref+".csyc").c_str());
  success &= (*logout) && (*sycout);
  active=true;

  if (!success) {
    cerr << "CoachLogger::create(): Probleme beim Erzeugen der Logfiles.\n";
    destroy();
    return false;
  }

  logout->precision (5);
  (*logout) << "FieldGeometry: " << REMBB.team_state.field_geometry.serialize() << '\n';
  Time nulltime;
  nulltime.set_usec (0);
  timeval tv;
  nulltime.get (tv);
  (*logout) << "StartingTimeval " << tv.tv_sec << ' ' << tv.tv_usec << '\n';
  return true;
}

std::ostream& CoachLogger::log_stream () throw () {
  return (active ? (*logout) : devnull);
}

void CoachLogger::log_cycleend () throw () {
  if (!active)
    return;

  Time now;
  unsigned long int timestamp = now.get_msec();
  for (unsigned int i=0; i<num_robots; i++) {
    bool isokay = REMBB.robot_state[i].comm_started && !REMBB.robot_state[i].comm_interrupted;
    if (isokay) {
      robotwriter[i]->write (timestamp, timestamp, REMBB.robot_state[i].robot_pos, timestamp, REMBB.robot_state[i].robot_pos);
      ballwriter[i]->write (timestamp, timestamp, REMBB.robot_state[i].ball_pos, timestamp, REMBB.robot_state[i].ball_pos);
    } else {
      RobotLocation rd;
      rd.pos=Vec(0.5*REMBB.team_state.field_geometry.field_length+5000, 0);
      BallLocation bd;
      bd.pos=Vec3D(0.5*REMBB.team_state.field_geometry.field_length+5000, 0, 0);
      robotwriter[i]->write (timestamp, timestamp, rd, timestamp, rd);
      ballwriter[i]->write (timestamp, timestamp, bd, timestamp, bd);
    }
    GameState gs;
    gs.refstate=REMBB.robot_state[i].refstate;
    gs.in_game=REMBB.robot_state[i].in_game;
    gamestatewriter[i]->write (timestamp, gs, REMBB.robot_state[i].playertype.c_str(), REMBB.robot_state[i].playerrole.c_str(), "");
    messageboardwriter[i]->write (timestamp, REMBB.robot_state[i].message_board);
  }
  if (REMBB.team_state.send_synch_signal)
    (*sycout) << timestamp << '\t' << REMBB.team_state.synch_signal << endl;
}

void CoachLogger::log_cyclebegin () throw () {
  if (active) {
    Time now;
    (*logout) << "\n%%%%cycle " << now << '\t' << ++cycle << endl;
  }
}
