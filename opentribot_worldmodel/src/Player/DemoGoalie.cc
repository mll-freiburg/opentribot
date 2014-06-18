
#include "DemoGoalie.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "../Behavior/Behaviors/Goalie/BGoalieBallInGoal.h"
#include "../Behavior/Behaviors/Goalie/BGoalieRaisedBall.h"
#include "../Behavior/Behaviors/Goalie/BGoalieGetAwayFromGoalPosts.h"
#include "../Behavior/Behaviors/Goalie/BGoaliePatrol.h"
#include "../Behavior/Behaviors/Goalie/BGoaliePositioning.h"
#include "../Behavior/Behaviors/Goalie/BGoalieAttackBall.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("DemoGoalie"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,std::bad_alloc) {
      return new DemoGoalie (reader);
    }
  };
  Builder the_builder;
}



DemoGoalie::DemoGoalie (const ConfigReader&) throw () : BehaviorPlayer ("DemoGoalie") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  
  Vec home_pos (0, -0.5*fg.field_length+10);
  Vec right_end (0.5*fg.goal_width-300, -0.5*fg.field_length);
  Vec attack_area1 (-0.5*fg.goal_width, home_pos.y-200);
  Vec attack_area2 (0.5*fg.goal_width, home_pos.y+500);
  
  addOption (new BGameStopped);
  addOption (new BGoalieBallInGoal);
  addOption (new BGoalieGetAwayFromGoalPosts);
  addOption (new BGoalieRaisedBall);
  addOption (new BGoalieAttackBall(false,attack_area1,attack_area2,true));
  addOption (new BGoaliePositioning (home_pos, right_end, Angle::deg_angle(1), false));
  addOption (new BGoaliePatrol (home_pos));
}

DemoGoalie::~DemoGoalie () throw () {;}

DriveVector DemoGoalie::process_drive_vector (Time texec) throw () {
  texec.add_msec(50);  // dann wird das Ueberschwingen geringer
  return BehaviorPlayer::process_drive_vector(texec);
}
