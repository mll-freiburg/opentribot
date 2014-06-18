
#include "FileWorldModel.h"
#include "../WorldModelFactory.h"
#include "../Prediction/update_robot_location.h"
#include "../../Structures/Journal.h"
#include "../../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("File"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType*) throw (TribotsException,bad_alloc) {
      return new FileWorldModel (reader);
    }
  };
  Builder the_builder;
}





FileWorldModel::FileWorldModel (const ConfigReader& read) throw (std::bad_alloc, Tribots::InvalidConfigurationException) : WorldModelTypeBase(read), rpos_stream(NULL), bpos_stream(NULL), opos_stream(NULL), gs_stream(NULL), robot_reader(NULL), ball_reader(NULL), obstacle_reader(NULL), gs_reader(NULL) {
  string fname;
  string fnamebase;
  if (read.get ("read_world_model_info", fnamebase)>0) {
    fname=fnamebase+".rpos";
    rpos_stream = new ifstream (fname.c_str());
    if (!(*rpos_stream)) {
      JERROR ("Robot position file: file error");
      delete rpos_stream;
      rpos_stream=NULL;
    } else {
      robot_reader = new RobotLocationReader (*rpos_stream);
    }
    fname=fnamebase+".bpos";
    bpos_stream = new ifstream (fname.c_str());
    if (!(*bpos_stream)) {
      JERROR ("Ball position file: file error");
      delete bpos_stream;
      bpos_stream=NULL;
    } else {
      ball_reader = new BallLocationReader (*bpos_stream);
    }
    fname=fnamebase+".opos";
    opos_stream = new ifstream (fname.c_str());
    if (!(*opos_stream)) {
      JERROR ("Obstacle file: file error");
      delete opos_stream;
      opos_stream=NULL;
    } else {
      obstacle_reader = new ObstacleLocationReader (*opos_stream);
    }
    fname=fnamebase+".gs";
    gs_stream = new ifstream (fname.c_str());
    if (!(*gs_stream)) {
      JERROR ("Game state file: file error");
      delete gs_stream;
      gs_stream=NULL;
    } else {
      gs_reader = new GameStateReader (*gs_stream);
    }
  }

  bloc.pos=Vec(0,0);
  bloc.velocity=Vec(0,0);
  bloc.pos_known=BallLocation::unknown;
}

FileWorldModel::~FileWorldModel () throw () {
  if (robot_reader) delete robot_reader;
  if (ball_reader) delete ball_reader;
  if (obstacle_reader) delete obstacle_reader;
  if (gs_reader) delete gs_reader;

  if (rpos_stream) delete rpos_stream;
  if (bpos_stream) delete bpos_stream;
  if (opos_stream) delete opos_stream;
  if (gs_stream) delete gs_stream;
}

ObstacleLocation FileWorldModel::get_obstacles (Time) const throw () {
  return obstacles;
}

RobotLocation FileWorldModel::get_robot (Time t) const throw () {
  return rpos_predict.get (t);
}

BallLocation FileWorldModel::get_ball (Time t) const throw () {
  BallLocation dest = bloc;
  dest.pos+=dest.velocity*(t.diff_msec(latest_bloc));
  return dest;
}

unsigned int FileWorldModel::update_localisation () throw () {
  Time now;
  unsigned long int t0, t1, t2;
  bool success;

  if (robot_reader) {
    RobotLocation rl1, rl2;
    success = robot_reader->read_until (t0, t1, rl1, t2, rl2, now.get_msec());
    if (success) {
      Time tt;
      tt.set_msec (t1);
      rpos_predict.set (flip_sides (rl1, get_own_half()), tt, tt);
    }
  }

  latest_vis_timestamp.set_msec (t1);

  if (ball_reader) {
    BallLocation bl1, bl2;
    success = ball_reader->read_until (t0, t1, bl1, t2, bl2, now.get_msec());
    if (success) {
      bloc = flip_sides (bl1, get_own_half());
      latest_bloc.set_msec(t1);
    }
  }

  if (obstacle_reader) {
    obstacles.clear();
    obstacle_reader->read_until (t0, obstacles, now.get_msec());
    if (get_own_half()<0)
      obstacles = flip_sides (obstacles, -1);
  }

  if (gs_reader) {
    GameState obj;
    unsigned long int t1;
    string playertype;
    string playerrole;
    string behavior;
    bool success=gs_reader->read_until (t1, obj, playertype, playerrole, behavior, now.get_msec());
    if (success) {
      startstop (obj.in_game);
      switch (obj.refstate) {
      case errorState: break;
      case stopRobot: update_refbox (SIGstop); break;
      case freePlay: update_refbox (SIGstart); break;
      case preOwnKickOff: update_refbox (SIGownKickOff); break;
      case preOpponentKickOff: update_refbox (SIGopponentKickOff); break;
      case preOwnFreeKick: update_refbox (SIGownFreeKick); break;
      case preOpponentFreeKick: update_refbox (SIGopponentFreeKick); break;
      case preOwnGoalKick: update_refbox (SIGownGoalKick); break;
      case preOpponentGoalKick: update_refbox (SIGopponentGoalKick); break;
      case preOwnCornerKick: update_refbox (SIGownCornerKick); break;
      case preOpponentCornerKick: update_refbox (SIGopponentCornerKick); break;
      case preOwnThrowIn: update_refbox (SIGownThrowIn); break;
      case preOpponentThrowIn: update_refbox (SIGopponentThrowIn); break;
      case preOwnPenalty: update_refbox (SIGownPenalty); break;
      case preOpponentPenalty: update_refbox (SIGopponentPenalty); break;
      case postOpponentKickOff: case postOpponentGoalKick:
      case postOpponentCornerKick: case postOpponentThrowIn:
      case postOpponentPenalty: case postOpponentFreeKick: update_refbox (SIGready); break;
      case ownPenalty: case opponentPenalty: update_refbox (SIGready); break;
      case preDroppedBall: update_refbox (SIGdroppedBall); break;
      case testState1: update_refbox (SIGtest1); break;
      case testState2: update_refbox (SIGtest2); break;
      case testState3: update_refbox (SIGtest3); break;
      case testState4: update_refbox (SIGtest4); break;
      case testState5: update_refbox (SIGtest5); break;
      case testState6: update_refbox (SIGtest6); break;
      case testState7: update_refbox (SIGtest7); break;
      case testState8: update_refbox (SIGtest8); break;
      }
    }
  }
  return 1|2|4;
}

void FileWorldModel::reset () throw () {;}

void FileWorldModel::reset (const Vec) throw () {;}

Time FileWorldModel::get_timestamp_latest_update () const throw () { return latest_vis_timestamp; }
