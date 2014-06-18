#include "WhiteBoard.h"
#include "../WorldModel/WorldModel.h"
#include "PassPlayer.h"
#include "PlayerFactory.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BallHandling/BPass.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallDirectly.h"
#include <cmath>

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("PassPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new PassPlayer (reader);
    }
  };
  Builder the_builder;
}

class BApproachBallDirectlyConditioned : public BApproachBallDirectly {
public:
  BApproachBallDirectlyConditioned() : BApproachBallDirectly("BApproachBallDirectlyConditioned") {}
  
  bool checkCommitmentCondition(const Time& t) throw() {
    return checkInvocationCondition(t);
  }

  bool checkInvocationCondition(const Time& t) throw() {
    const Vec& ballLocation = MWM.get_ball_location(t).pos.toVec();
    //const FieldGeometry& field = MWM.get_field_geometry();

    Vec relBall=WBOARD->getAbs2RelFrame(t) * ballLocation;
    if (fabs(relBall.x) < 300 && fabs(relBall.y) < 600) {
      return true;
    }
    return false;
  }
};



PassPlayer::PassPlayer (const ConfigReader& reader) throw ()
  : BehaviorPlayer ("PassPlayer") 
{
  WBOARD->readConfigs (reader);
  WBOARD->checkMessageBoard(); // important for getting ownsball

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  XYRectangle area(Vec( fgeom.field_width/2.  + 500., 
                        fgeom.field_length/2. + 500.),
                   Vec(-fgeom.field_width/2.  - 500.,
                       -fgeom.field_length/2. - 500)); 
  Vec target(0.0, fgeom.field_length/4.);
  Vec startPos(fgeom.field_width/3., -fgeom.field_length/3.);
	Vec startHeading(0.0, 1.0);

  addOption(new BGameStopped());
  addOption(new BStayInsideArea(area, target)); // Training
  addOption(new BPass(1.0)); //pass probability
  addOption(new BApproachBallDirectlyConditioned());
  addOption(new BGotoPosEvadeObstacles(startPos, startHeading)); // go home, stdspeed=1.3
  addOption(new BEmergencyStop());
}

