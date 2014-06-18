
#include "DistanceCalibrationPlayer.h"
#include "../../Player/PlayerFactory.h"
#include "../../WorldModel/WorldModel.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;


DistanceCalibrationPlayer::DistanceCalibrationPlayer () throw () {
  mode=1;
}

DriveVector DistanceCalibrationPlayer::process_drive_vector (Time t) throw () {
  DriveVector dv (Vec(0,0), 0, false);
  // Phasen: 
  // 1. nichts tun, bis Roboter aktiviert wird
  // 2. fuenf Sekunden warten
  // 3. 22 Sekunden lang drehen
  // 4. nichts mehr tun
  if (mode==1) {
    if (MWM.get_game_state().in_game) {
      timer.update();
      mode=2;
    }
    return dv;
  }
  if (mode==2) {
    if (timer.elapsed_msec()>=5000) {
      timer.update();
      mode=3;
    }
    return dv;
  }
  if (mode==3) {
    if (timer.elapsed_msec()>=22000) {
      mode=4;
    }
    dv.vrot = 0.6;
    return dv;
  }
  return dv;
}


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("DistanceCalibrationPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader&, PlayerType*) throw (TribotsException,bad_alloc) {
      return new DistanceCalibrationPlayer ();
    }
  };
  Builder the_builder;
}
