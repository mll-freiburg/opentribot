#include "BTouchBallAfterStandard.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Structures/TeammateLocation.h"
#include <vector>
#include <cmath>
#include <sstream>

using namespace std;
namespace Tribots
{

BTouchBallAfterStandard::
BTouchBallAfterStandard()
  : Behavior("BTouchBallAfterStandard"),
    goToBall(new BApproachBallFromBehindPointingToMiddle(Vec(0,-4000)))
    {
       opponentCanExecuteStandard = true;
       counter = 0;
       passed = false; 
       start.update();
       start.add_sec(-50);
    }

BTouchBallAfterStandard::
~BTouchBallAfterStandard() throw ()
{
}

void 
BTouchBallAfterStandard::cycleCallBack (const Time& t) throw()
{
  bool pres = (MWM.get_game_state().refstate == postOpponentThrowIn    ||
               MWM.get_game_state().refstate == postOpponentGoalKick   ||
               MWM.get_game_state().refstate == postOpponentCornerKick ||
               MWM.get_game_state().refstate == postOpponentFreeKick   ||
               MWM.get_game_state().refstate == postOpponentKickOff);
  
  bool last = (lastGameState == postOpponentThrowIn    ||
               lastGameState == postOpponentGoalKick   ||
               lastGameState == postOpponentCornerKick ||
               lastGameState == postOpponentFreeKick   ||
               lastGameState == postOpponentKickOff);
  
  if (!last && pres) {  // dies ist der erste interessante zustand (post gerade angeschaltet)
    start=t;
  } 
  
  if (pres) {
    lastGameState = MWM.get_game_state().refstate;
    counter = 5;// remember for 5 cylces, start overwriting afterwards again
  }else{
    if(counter <= 0){
      lastGameState = MWM.get_game_state().refstate;
    }
    counter--;
  }
}

bool 
BTouchBallAfterStandard::
checkCommitmentCondition(const Time& t) throw()
{
  return 
    MWM.get_game_state().refstate == freePlay && (!passed || counter2 < 90) &&
    ! WBOARD->teamPossessBall() && !WBOARD->touchedBall();
}

bool 
BTouchBallAfterStandard::
checkInvocationCondition(const Time& t) throw()
{
  return (!opponentCanExecuteStandard) &&
         (WBOARD->getZonePressureRole() == "ballL" ||
          WBOARD->getZonePressureRole() == "ballR") &&
         (lastGameState == postOpponentThrowIn ||
          lastGameState == postOpponentGoalKick ||
          lastGameState == postOpponentCornerKick ||
          lastGameState == postOpponentFreeKick ||
          lastGameState == postOpponentKickOff) &&
         MWM.get_game_state().refstate == freePlay &&
         t.diff_sec(start) >= 8;   // mindestens 8s seit start vergangen, sonst vermutlich ausgeführt.
}

DriveVector 
BTouchBallAfterStandard::getCmd(const Time& t) 
  throw(TribotsException) 
{
    const FieldGeometry& field = MWM.get_field_geometry();
    const std::vector<TeammateLocation>& teammates = MWM.get_teammate_location();
    unsigned int robotSelfId = MWM.get_robot_id();
    unsigned int targetId;
    const RobotLocation& robot = MWM.get_robot_location(t);
    Vec target(0,field.field_length/2-4000);
    DriveVector dv;
    if(WBOARD->doPossessBall(t) && !passed){
      unsigned int closestId = 0;
      for (unsigned int i=1; i < teammates.size(); i++) {
        if(teammates[i].number == robotSelfId)
          continue;
        if ((robot.pos-teammates[closestId].pos).length() >
          (robot.pos-teammates[i].pos).length()) {
          closestId = i;
        }
      }
      targetId = teammates[closestId].number;
      std::stringstream msg;
      msg << "pass: " << targetId;
      MWM.get_message_board().publish(msg.str());
      MWM.get_message_board().publish("TochedBall:");
      LOUT << "Touch Pass zu Spieler " << targetId << "\n";
      WBOARD->resetPossessBall(); // Ballbesitz durch pass verloren
      counter2 = 0;
      passed = true;
    }
    if(passed){
      dv.vtrans = ((robot.pos - target).normalize()) / robot.heading; // 1m/s
      dv.vrot = 0;
      dv.kick = 0;
      counter2++;
      return dv;
    }
    return goToBall->getCmd(t);
}
  
  
void 
BTouchBallAfterStandard::gainControl(const Time& t) 
  throw(TribotsException)
{
  counter2 = 0;
  passed = false;
}

void 
BTouchBallAfterStandard::loseControl(const Time& t) 
  throw(TribotsException)
{
  counter = 0;
  passed = false;  
}

void BTouchBallAfterStandard::updateTactics (const TacticsBoard& tb) throw() {
  string key = "OpponentCanExecuteStandard";
  if(tb[key] == string("an")){
    opponentCanExecuteStandard = true;
  }else if(tb[key] == string("aus")){
    opponentCanExecuteStandard = false;
  }
}

};
