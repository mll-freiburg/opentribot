
#include "PlayerDummy.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "../Structures/GameState.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("Dummy"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new PlayerDummy (reader);
    }
  };
  Builder the_builder;
}




PlayerDummy::PlayerDummy (const ConfigReader& reader) throw () {
  vtrans=Vec(0,0);
  vrot=0;
  for (int i=0; i<3; i++) vaux[i] = 0;
  dv_mode = ROBOTVELOCITY;

  int h;
  if (reader.get("PlayerDummy::DriveVectorMode", h)) {
    switch (h) {
    case 0: dv_mode = ROBOTVELOCITY; break;
    case 1: dv_mode = WHEELVELOCITY; break;
    case 2: dv_mode = MOTORVOLTAGE; break;
    default:
      dv_mode = ROBOTVELOCITY;
    }
  }

  vector<double> dd (2);
  double d;
  if (reader.get("PlayerDummy::vtrans", dd)>=2) {
    vtrans.x=dd[0];
    vtrans.y=dd[1];
  }
  if (reader.get("PlayerDummy::vrot", d)>=1)
    vrot=d;
  
  vector<double> ddd (3);
  if (reader.get("PlayerDummy::vaux", ddd)>=3) {
    for (int i=0; i<3; i++) vaux[i]=ddd[i]; 
  }
}

DriveVector PlayerDummy::process_drive_vector (Time tt) throw () {

  MWM.get_message_board().publish("Dummy an Erde");

  if (MWM.get_game_state().refstate==stopRobot)
    return DriveVector (Vec(0,0),0,false);
  else {
    if (dv_mode == ROBOTVELOCITY)
      return DriveVector (vtrans,vrot,false);
    return DriveVector(vaux[0], vaux[1], vaux[2], false, dv_mode);
  }
}
