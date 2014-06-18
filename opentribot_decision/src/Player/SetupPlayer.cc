
#include "SetupPlayer.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/Setup/BTestStateStuckStop.h"
#include "../Behavior/Behaviors/Setup/BTestAllDirections.h"
#include "../Behavior/Behaviors/Setup/BNewDribbleTest.h"
#include "../Behavior/Behaviors/Setup/BSecurity.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallDirectly.h"
#include "../Behavior/Skills/BallHandling/SDribbleBallToPos.h"
#include "../Behavior/SPBehavior.h"
#include "../Behavior/Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include "../Structures/Journal.h"
#include "../Fundamental/geometry.h"
#include <sstream>

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("SetupPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new SetupPlayer (reader);
    }
  };
  Builder the_builder;

  const char* setupPlayerRoles [6] = {
    "default",
    "accept",
    "reject",
    "test",
    "dribbleLeftArea",
    "dribbleRightArea"
  };

}

BStopWhenRequested::BStopWhenRequested () throw () { name="StopWhenRequested"; }
BStopWhenRequested::~BStopWhenRequested () throw () {;}
bool BStopWhenRequested::checkInvocationCondition(const Time&) throw() { return active; }
bool BStopWhenRequested::checkCommitmentCondition(const Time&) throw() { return active; }
void BStopWhenRequested::setActive (bool b) { active=b; };

namespace {

  class BApproachBallDirectlyConditioned : public BApproachBallDirectly {
  public:
    BApproachBallDirectlyConditioned () {
      name = "BApproachBallDirectlyConditioned";
    }
    bool checkInvocationCondition (const Time& t) throw() {
      return MWM.get_ball_location(t).pos_known==BallLocation::known;
    }
    bool checkCommitmentCondition (const Time& t) throw() {
      return MWM.get_ball_location(t).pos_known==BallLocation::known &&
          !WBOARD->doPossessBall(t);
    }
  };
  class BDribbleToPosition : public Behavior {
    SDribbleBallToPos* controller;
    Vec target;
  public:
    BDribbleToPosition (Vec tp) : Behavior ("BDribbleToPosition"), target(tp) {
      controller = new SDribbleBallToPos;
      controller -> setParameters (tp, 2.5, true);
      stringstream inout;
      inout << "BDribbleToPosition_" << target.x << '_' << target.y << '\n';
      getline (inout, name);
    }
    ~BDribbleToPosition () throw () {
      delete controller;
    }
    void setTarget (Vec p) throw () {
      target=p;
      controller -> setParameters (target, 2.5, true);
    }
    bool checkCommitmentCondition(const Time& t) throw() {
      return WBOARD->doPossessBall(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return WBOARD->doPossessBall(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException) {
      DriveVector dv=controller->getCmd (t);
      dv.kick=false;
      return dv;
    }
  };

  class BApproachAndDribbleToPos : public BDIBehavior {
  Vec target;
  BDribbleToPosition* dribble;
  public:
    BApproachAndDribbleToPos (Vec tg) : target(tg) {
      name="BApproachAndDribbleToPos";
      dribble=new BDribbleToPosition (target);
      addOption (dribble);
      addOption (new BApproachBallDirectlyConditioned);
    }
    void setTarget (Vec tg) { target=tg; dribble->setTarget (tg); }
    bool checkCommitmentCondition(const Time& t) throw() {
      return MWM.get_ball_location(t).pos_known==BallLocation::known &&
          abs(MWM.get_ball_location(t).pos.x)<0.5*MWM.get_field_geometry().field_width-1000 &&
          MWM.get_ball_location(t).pos.y<0 &&
          abs(MWM.get_ball_location(t).pos.y)<0.5*MWM.get_field_geometry().field_length-1000 &&
          (MWM.get_robot_location(t).pos-target).length()>400;
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return MWM.get_ball_location(t).pos_known==BallLocation::known &&
          abs(MWM.get_ball_location(t).pos.x)<0.5*MWM.get_field_geometry().field_width-2000 &&
          MWM.get_ball_location(t).pos.y<-1000 &&
          abs(MWM.get_ball_location(t).pos.y)<0.5*MWM.get_field_geometry().field_length-2000;
    }
  };

  class BJustKick : public Behavior {
  public:
    BJustKick () : Behavior ("BJustKick") {;}
    bool checkCommitmentCondition(const Time&) throw() {
      return false;
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return WBOARD->doPossessBall(t) && MWM.get_ball_location(t).pos.y<-MWM.get_field_geometry().field_length/2+MWM.get_field_geometry().penalty_area_length;
    }
    DriveVector getCmd(const Time&) throw(TribotsException) {
      DriveVector dv = MWM.get_recent_drive_vector();
      dv.kick=true;
      return dv;
    }
    void cycleCallBack(const Time& t) throw() {
      WBOARD->checkMessageBoard();
    }
  };

  class BGotoPosX : public Behavior {
    SPhysGotoPosAvoidObstacles skill;
    Vec targetPosition;
    Angle targetHeading;
  public:
    BGotoPosX (Vec p, Angle h) : Behavior ("GotoPos"), targetPosition(p), targetHeading(h) {
      skill.init (targetPosition, targetHeading, true);
      skill.set_dynamics (1.5);
    }
    ~BGotoPosX () throw () {;}
    bool checkCommitmentCondition(const Time& t) throw() {
      const RobotLocation robot (MWM.get_robot_location(t));
      return (robot.pos-targetPosition).length()>200 || !(targetHeading-robot.heading).in_between(Angle::deg_angle(-10), Angle::deg_angle(10));
    }
    bool checkInvocationCondition(const Time&) throw() {
      return true;
    }
    DriveVector getCmd(const Time& t) throw(TribotsException) {
      return skill.getCmd(t);
    }
  };

  class BDribbleAndKickBehavior : public SPBehavior {
    RefereeState teststate;
  public:
    BDribbleAndKickBehavior (RefereeState rs) : SPBehavior ("BDribbleAndKickBehavior"), teststate(rs) {
      appendStage (new BGotoPosX (Vec(-2000, -2000), -Angle::quarter));
      appendStage (new BGotoPosX (Vec(2000, -2000), -Angle::quarter));
      appendStage (new BGotoPosX (Vec(2000, -2000), -Angle::quarter+Angle::deg_angle(120)));
      appendStage (new BGotoPosX (Vec(-2000, -2000), -Angle::quarter+Angle::deg_angle(120)));
      appendStage (new BGotoPosX (Vec(-2000, -2000), -Angle::quarter-Angle::deg_angle(120)));
      appendStage (new BGotoPosX (Vec(2000, -2000), -Angle::quarter-Angle::deg_angle(120)));
      appendStage (new BApproachAndDribbleToPos (Vec(-1000, -3000)));
      appendStage (new BApproachAndDribbleToPos (Vec(1000, -3000)));
      appendStage (new BApproachAndDribbleToPos (Vec(-1000, -4000)));
      appendStage (new BApproachAndDribbleToPos (Vec(0,-MWM.get_field_geometry().field_length/2)), false, true);
      appendStage (new BJustKick);
      appendStage (new BEmergencyStop);
    }
    bool checkCommitmentCondition(const Time& t) throw() {
      return (MWM.get_game_state().refstate==teststate) && SPBehavior::checkCommitmentCondition (t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return (MWM.get_game_state().refstate==teststate) && SPBehavior::checkInvocationCondition (t);
    }
  };

  class BDribbleTest2 : public BDIBehavior {
    SPBehavior* sp;
    BApproachAndDribbleToPos* p1 [20];
    BApproachAndDribbleToPos* p2 [20];
    RefereeState teststate;
    XYRectangle ICarea;
    XYRectangle CCarea;
  public:
    bool checkCommitmentCondition(const Time& t) throw() {
      return (MWM.get_game_state().refstate==teststate) && MWM.get_ball_location(t).pos_known==BallLocation::known && ICarea.is_inside(MWM.get_ball_location(t).pos.toVec());
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return (MWM.get_game_state().refstate==teststate) && MWM.get_ball_location(t).pos_known==BallLocation::known && CCarea.is_inside(MWM.get_ball_location(t).pos.toVec()); 
    }
    void setPlayerRole (const char* newrole) throw() {
      const FieldGeometry& field (MWM.get_field_geometry());
      if(std::string(newrole)=="dribbleLeftArea") {
        ICarea = XYRectangle (Vec(-field.field_width/2+1000, -field.field_length/2+1000), Vec (-1000, -1000));
        CCarea = XYRectangle (Vec(-field.field_width/2+500, -field.field_length/2+500), Vec (-500, -500));
        for (unsigned int i=0; i<20; i++) {
          p1[i]->setTarget (Vec(-field.field_width/4, -field.field_length/6-1000));
          p2[i]->setTarget (Vec(-field.field_width/4, -field.field_length/3+1000));
        }
      } else if (std::string(newrole)=="dribbleRightArea") {
        ICarea = XYRectangle (Vec (1000, -1000), Vec(field.field_width/2-1000, -field.field_length/2+1000));
        CCarea = XYRectangle (Vec (500, -500), Vec(field.field_width/2-500, -field.field_length/2+500));
        for (unsigned int i=0; i<20; i++) {
          p1[i]->setTarget (Vec(field.field_width/4, -field.field_length/6-1000));
          p2[i]->setTarget (Vec(field.field_width/4, -field.field_length/3+1000));
        }
      } else if (std::string(newrole)=="default") {
        ICarea = XYRectangle (Vec (-2500, -1000), Vec(2500, -field.field_length/2+1000));
        CCarea = XYRectangle (Vec (-3000, -500), Vec(3000, -field.field_length/2+500));
        for (unsigned int i=0; i<20; i++) {
          p1[i]->setTarget (Vec(0, -field.field_length/6-1000));
          p2[i]->setTarget (Vec(0, -field.field_length/3+1000));
        }
      }
    }
    BDribbleTest2 (RefereeState ts) : BDIBehavior ("BDribbleTest2"), teststate (ts) {
      sp=new SPBehavior;
      addOption (sp);
      for (unsigned int i=0; i<20; i++) {
        p1[i]=new BApproachAndDribbleToPos (Vec(0,0));
        p2[i]=new BApproachAndDribbleToPos (Vec(0,0));
      }
      for (unsigned int i=0; i<20; i++) {
        sp->appendStage (p1[i]);
        sp->appendStage (p2[i]);
      }
      setPlayerRole ("default");
    }
  };

  class BDribbleTest : public BDIBehavior {
    RefereeState teststate;
    BNewDribbleTest* bNewDribbleTest;
  public:
    bool checkCommitmentCondition(const Time&) throw() { return MWM.get_game_state().refstate==teststate; }
    bool checkInvocationCondition(const Time&) throw() { return MWM.get_game_state().refstate==teststate; }
    virtual void setPlayerRole (const char* newrole) throw() {
      if(std::string(newrole)=="dribbleLeftArea" || std::string(newrole)=="dribbleRightArea"){
        bNewDribbleTest->set_field_area(std::string(newrole));
      }
    }
    BDribbleTest (RefereeState rs) : teststate (rs) {
      name="BDribbleTest";
      bNewDribbleTest = new BNewDribbleTest(2.0);
      addOption (new BSecurity());
      addOption (new BStayInsideArea(*(bNewDribbleTest->get_dribble_area()),
                 *(bNewDribbleTest->get_dribble_area_center())));
      addOption (bNewDribbleTest);
      addOption (new BApproachBallDirectly());
      addOption (new BEmergencyStop());
    }
  };

}

SetupPlayer::SetupPlayer (const ConfigReader& cfg) throw ()
  : BehaviorPlayer ("SetupPlayer", setupPlayerRoles, 6)
{
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);
  cfg.get ("WhiteBoard::OwnsBallPixelDistance", ownsBallDistance);
  oldOwnsBallDistance=ownsBallDistance;
  cfgFilename="";
  const vector<string>& sources (cfg.list_of_sources());
  if (sources.size()>0) {
    cfgFilename=sources[0];
    JMESSAGE (cfgFilename.c_str());
  }


  bOwnsBallCheck  = new BTestOwnsBallCheck(testState1);
  bStop = new BStopWhenRequested();

  addOption (new BGameStopped());
  addOption (new BTestStateStuckStop());
  addOption (bStop);
  addOption (bOwnsBallCheck);
  addOption (new BDribbleTest(testState2));
  addOption (new BTestAllDirections(testState3));
  addOption (new BDribbleTest2(testState4));
  addOption (new BDribbleAndKickBehavior(testState8));
  addOption (new BEmergencyStop());

  set_role("default");
}

SetupPlayer::~SetupPlayer () throw () {
}

bool SetupPlayer::set_role (const char* newrole) throw () {
  bStop->setActive (false);
  if (std::string(newrole)=="reject") {
    bOwnsBallCheck->clearCalibration ();
    ConfigReader cfg2;
    cfg2.set ("WhiteBoard::OwnsBallPixelDistance", oldOwnsBallDistance);
    WBOARD->readConfigs (cfg2);
    ownsBallDistance=oldOwnsBallDistance;
    return BehaviorPlayer::set_role ("reject");
  } else if (std::string(newrole)=="default") {
    return BehaviorPlayer::set_role ("default");
  }else if(std::string(newrole)=="dribbleLeftArea" || std::string(newrole)=="dribbleRightArea"){
    return BehaviorPlayer::set_role (newrole);
  }
  int newDistance = static_cast<int>(bOwnsBallCheck->getCalibration());
  if (newDistance<=0)
    return false;
  if (std::string(newrole)=="test") {
    bStop->setActive (true);
    ConfigReader cfg2;
    cfg2.set ("WhiteBoard::OwnsBallPixelDistance", newDistance);
    oldOwnsBallDistance=ownsBallDistance;
    ownsBallDistance=newDistance;
    WBOARD->readConfigs (cfg2);
    return BehaviorPlayer::set_role("test");
  } else if (std::string(newrole)=="accept") {
    ConfigReader cfg2;
    cfg2.append_from_file (cfgFilename.c_str());
    ownsBallDistance=newDistance;
    cfg2.set ("WhiteBoard::OwnsBallPixelDistance", newDistance);
    WBOARD->readConfigs (cfg2);
    cfg2.replace_config_file (cfgFilename.c_str());
    stringstream inout;
    inout << "Change WhiteBoard::OwnsBallPixelDistance from " << oldOwnsBallDistance << " to " << ownsBallDistance << '\n';
    std::string message;
    getline (inout, message);
    JMESSAGETS(message.c_str());
    oldOwnsBallDistance=ownsBallDistance;
    return BehaviorPlayer::set_role (newrole);
  } else {
    return false;
  }
}
