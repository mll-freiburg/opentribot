#include "BPostOpponentStandardSituation.h"
#include "../../../WorldModel/WorldModel.h"

namespace Tribots {
  
  using namespace std;

  BPostOpponentStandardSituation::BPostOpponentStandardSituation()
    : Behavior("BPostOpponentStandardSituation"), waitingTime(100), communicateBallMoved(true)
  { tLosSend = Time();
    tLosSend.add_sec(-20);
    lastBallMoved = Time();
    lastBallMoved.add_sec(-100);
    lastGameState = MWM.get_game_state().refstate;
  }

  BPostOpponentStandardSituation::~BPostOpponentStandardSituation() throw ()
  {}
  
  void BPostOpponentStandardSituation::cycleCallBack(const Time& t) throw() 
  {
    if (lastGameState != MWM.get_game_state().refstate &&
        (MWM.get_game_state().refstate == postOpponentThrowIn ||
         MWM.get_game_state().refstate == postOpponentGoalKick ||
         MWM.get_game_state().refstate == postOpponentCornerKick ||
         MWM.get_game_state().refstate == postOpponentFreeKick ||
         MWM.get_game_state().refstate == postOpponentKickOff)) {
      tLosSend = t;
    }
    lastGameState = MWM.get_game_state().refstate;
  }
  
  void BPostOpponentStandardSituation::updateTactics (const TacticsBoard& tb) throw ()
  {
    string key = "StandardSituationWartezeit";
    if (tb[key] == string("1")) {
      waitingTime = 1;
    }
    else if (tb[key] == string("2")) {
      waitingTime = 2;
    }
    else if (tb[key] == string("4")) {
      waitingTime = 4;
    }
    else if (tb[key] == string("6")) {
      waitingTime = 6;
    }
    else if (tb[key] == string("8")) {
      waitingTime = 8;
    }
    else if (tb[key] == string("10")) {
      waitingTime = 10;
    }
    else {      // automatisch      auf Weltmodell vertrauen
      waitingTime = 100;
    }

    if (tb[string("StandardSituationSendeBall")]==string("aus")) {
      communicateBallMoved=false;
    }
    else {
      communicateBallMoved=true;
    }
  }
  
  void BPostOpponentStandardSituation::gainControl(const Time& t) throw(TribotsException)
  {}
  
  bool 
  BPostOpponentStandardSituation::checkCommitmentCondition(const Time& t) 
    throw()
  {
      return BPostOpponentStandardSituation::checkInvocationCondition(t);
  }

  bool 
  BPostOpponentStandardSituation::checkInvocationCondition(const Time& t) 
    throw()
  {
//    const BallLocation& ball = MWM.get_ball_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);
    const FieldGeometry& fgeom = MWM.get_field_geometry();

//    LOUT << "BPOppStSi: " << lastBallMoved << " " << tLosSend << " " << t 
//      << " " << (!(lastBallMoved > tLosSend && t.diff_sec(lastBallMoved) < 10)  ||
//                 robot.pos.y < -fgeom.field_length/2. + 300. + fgeom.penalty_area_length) 
//      << " " << t.diff_sec(tLosSend) << endl;
    
    return 
      (MWM.get_game_state().refstate == postOpponentThrowIn ||
       MWM.get_game_state().refstate == postOpponentGoalKick ||
       MWM.get_game_state().refstate == postOpponentCornerKick ||
       MWM.get_game_state().refstate == postOpponentFreeKick ||
       MWM.get_game_state().refstate == postOpponentKickOff) &&
      (!(lastBallMoved > tLosSend && t.diff_sec(lastBallMoved) < 10)  ||
       robot.pos.y < -fgeom.field_length/2. + 300. + fgeom.penalty_area_length) &&  // muss selber entscheiden, wenn auf blockposition im oder am strafraum
      t.diff_sec(tLosSend) < waitingTime;
  }

  DriveVector 
  BPostOpponentStandardSituation::getCmd(const Time& t) throw(TribotsException)
  {
    if (MWM.get_message_board().scan_for_prefix ("BallMoved!").length() > 0) {
      lastBallMoved.update();
    }
    
    DriveVector dv;
    dv.kick = 0;
    dv.vrot = 0;
    dv.vtrans = Vec(0.,0.);
    return dv;
  }
  
  void BPostOpponentStandardSituation::loseControl(const Time&) throw(TribotsException) {
    if (communicateBallMoved) MWM.get_message_board().publish("BallMoved!");
  }
  
  

}
