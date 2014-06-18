#include <stdlib.h>
#include "DribbleTestPlayer.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "PlayerFactory.h"
#include "../Fundamental/RemoteTune.h"
using namespace Tribots;
using namespace std;


namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("DribbleTestPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader,
                            PlayerType*)
      throw (TribotsException,std::bad_alloc)
    {
      return new DribbleTestPlayer (reader);
    }
  };
  Builder the_builder;
}


// --- BEGIN implementation BDribbleTest ---
BDribbleTest::BDribbleTest(double transVel)
  :Behavior("BDribbleTest") {
  
  skill = new SDribbleBallToPos();
  
  timer = 0;
  
  currentWayPoint = 0;
 // addWayPoint(Vec(-2000,-2000));
 // addWayPoint(Vec(-2000,2000));

 /** 
  addWayPoint(Vec(2000.,1000));
  addWayPoint(Vec(-2000.,1000));
 
  addWayPoint(Vec(-1500.,-2000));
  addWayPoint(Vec(1500.,-2000));
**/
  
  
  addWayPoint(Vec(0.,2500));
  addWayPoint(Vec(0.,-2500));
  addWayPoint(Vec(0.,2500));
  addWayPoint(Vec(0.,-2500));
  addWayPoint(Vec(0.,1500));
  addWayPoint(Vec(0.,-1500));
  addWayPoint(Vec(0.,1500));
  addWayPoint(Vec(0.,-500));
  addWayPoint(Vec(0.,500));
  addWayPoint(Vec(0.,-500));
   addWayPoint(Vec(1000,3000));

   
  //   addWayPoint(Vec(-1500,2000));
 //   addWayPoint(Vec(1500,2000));
   
   
  dribblevel = 2.0;   // default: 1.8
  }

BDribbleTest::~BDribbleTest() throw () {
  delete skill;
}

bool BDribbleTest::checkCommitmentCondition(const Time& t) throw() {
  timer++;
  //if (! WBOARD->doPossessBall(t)) {
  //  return false;
  //}  
  return true;    
}

bool BDribbleTest::checkInvocationCondition(const Time& t) throw() {
  //if (! WBOARD->doPossessBall(t)) {
  //  return false;
  //}
  return true; 
}

DriveVector BDribbleTest::getCmd(const Time& t) throw(TribotsException) {
  
  TUNABLE("DRibble_Vel",&dribblevel);
  LOUT << "###Tuningactual dribblespeed/dribblevel :" << dribblevel ;
  LOUT << "TIMER: "<<timer<<endl;
  
  world2robot	= WBOARD->getAbs2RelFrame(t);
  robot2world	= WBOARD->getRel2AbsFrame(t);    
  robot				= MWM.get_robot_location(t);
  
  if (wayPoints[currentWayPoint].area.is_inside(robot.pos)) {
    if (currentWayPoint==(int)wayPoints.size()-1) currentWayPoint = 0;
    else currentWayPoint++;
  }
  
  LOUT<<"driving to WayPoint #"<<currentWayPoint<<endl;
  wayPoints[currentWayPoint].area.draw(LOUT);
  
  skill->setParameters(wayPoints[currentWayPoint].pos, dribblevel , 0);
  return skill->getCmd(t);
}

void BDribbleTest::updateTactics (const TacticsBoard& tb) throw ()
{
  string velstr="";
  velstr = tb[string("DribbleTestVel")];
  if (velstr != "") {
    dribblevel = atof(velstr.c_str()); 
  }
}

void BDribbleTest::gainControl(const Time& t) throw(TribotsException) {
  timer=0;
}

void BDribbleTest::loseControl(const Time& t) throw(TribotsException) {
  timer=0;
}
    
void BDribbleTest::addWayPoint(Vec pos) {
  struct wayPoint _wP;
  _wP.pos=pos;
  _wP.area=XYRectangle(Vec(pos.x-500,pos.y-500),Vec(pos.x+500,pos.y+500));
  wayPoints.push_back(_wP);
}

// --- END implementation BDribbleTest ---



DribbleTestPlayer::DribbleTestPlayer(const ConfigReader&) throw ()
  : BehaviorPlayer ("DribbleTestPlayer", 0 , 0)
{
  
   addOption (new BGameStopped());
   
   addOption (new BDribbleTest( 1.8 ));

}

DribbleTestPlayer::~DribbleTestPlayer() throw ()
{
  ;
}
