#include <cmath>
#include "FieldPlayerStuttgart.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Fundamental/geometry.h"
#include "../WorldModel/WorldModel.h"
#include "../WorldModel/FataMorgana.h"
#include "../Behavior/Skills/WithoutBall/SPatrol.h"
#include "../Behavior/SPBehavior.h"
#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.h"
#include "../Behavior/Behaviors/ApproachingBall/BVolleyApproach.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BEindhoven.h"
#include "../Behavior/Behaviors/BallHandling/BBreakAttack.h"
#include "../Behavior/Behaviors/BallHandling/BShakeOffDefender.h"
#include "../Behavior/Behaviors/BallHandling/BRetreatDribble.h"
#include "../Behavior/Behaviors/BallHandling/BWingAttack.h"
#include "../Behavior/Behaviors/BallHandling/BPassSpontaneously.h"
#include "../Behavior/Behaviors/BallHandling/BBefreiungsschlag.h"
#include "../Behavior/Behaviors/BallHandling/BTouchBallAfterStandard.h"
#include "../Behavior/Behaviors/BallHandling/BEigenMove.h"
#include "../Behavior/Behaviors/BallHandling/BStuckOwnsBall.h"
#include "../Behavior/Behaviors/BallHandling/BStuckDistanceShooter.h"
#include "../Behavior/Behaviors/BallHandling/BBoostBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BPassBeforeGoal.h"
#include "../Behavior/Behaviors/BasicMovements/BShootEmergency.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BDraufhalten.h"
#include "../Behavior/Behaviors/BasicMovements/BFarShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.h"
#include "../Behavior/Behaviors/WithoutBall/BSupportLongPass.h"
#include "../Behavior/Behaviors/WithoutBall/BSupportNearBall.h"
#include "../Behavior/Behaviors/WithoutBall/BCounterAttack.h"
#include "../Behavior/Behaviors/WithoutBall/BBlockWayToGoal.h"
#include "../Behavior/Behaviors/WithoutBall/BPreventInterferenceWithPass.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/SpecialGameStates/BTestStateStop.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituationNew.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPostOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOwnIndirectStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BOwnPenalty.h"
#include "../Behavior/Behaviors/ZonePressure/BZonePressure.h"
#include "../Behavior/Behaviors/WithoutBall/BLeaveGoal.h"
#include "../Behavior/Behaviors/WithoutBall/BOpposeBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BInterceptBallStatic.h"
#include "../Behavior/Behaviors/ApproachingBall/BCatchBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BTurnAroundPos.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallStatic.h"


#include "../Libs/Alib/AssiLib/AssiLib.h"

//#include "../tools/BlackBoard/client/bbclient.h"
#include "../tools/StuttgartListener/stuttgartinterface.h"

// Innerhalb dieser Datei wird die spielertypspezifische Strategie des 
// Feldspielers festgelegt. Die vom Feldspieler verwendeten Behaviors und
// Skills sollten soweit wie m�lich generisch gehalten werden, d.h. 
// Strategiespezifische Einstellungen wie Aktionsbereiche und 
// Rollenspezifikationen sollten aussschlie�ich hier ber das setzen von 
// allgemeinen Parametern der Behaviors und durch Ableiten und �erschreiben 
// ihrer Aktivierungsbedingungen vorgenommen werden.


#define REFSTATE MWM.get_game_state().refstate
#include "../Behavior/Behaviors/ZonePressure/BProtectGoal.h"
#include "../Behavior/Behaviors/ZonePressure/BSafety.h"


using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("FeldspielerStuttgart"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, 
			    PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new HumanCoopPlayer (reader);
    }
  };
  Builder the_builder;
}

namespace Tribots {   // unbenannter Namespace notwenig, um Verwechslungen beim Linken mit FieldPlayer zu vermeiden

static string FieldPlayerStuttgart_role = "ballL";
static StuttgartInterface * stuttpointer;

//
// BPatrol verwendet den Skill SPatrol mit geeigneten Patrolpositionen,
// die von der aktuellen Rolle abgeleitet werden.
//
class BPatrolStuttgart : public Behavior {
public:
 
  BPatrolStuttgart() : Behavior("BPatrolStuttgart"), patrol(new SPatrol()) {};
  ~BPatrolStuttgart() throw() { delete patrol; }
  
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    
    return (ball.pos_known == BallLocation::unknown); 
  }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return BPatrolStuttgart::checkCommitmentCondition(t);
  }
  virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
    const FieldGeometry& field = MWM.get_field_geometry();      
      
        std::vector<Vec> positions;
        positions.push_back(Vec(0,-field.field_length/2+field.penalty_marker_distance));
        positions.push_back(Vec(-field.field_width*2/6,0));
        positions.push_back(Vec(field.field_width*2/6,0));
        patrol->setPatrolPositions(Vec(0.,0.),Vec(0.,0.));
        patrol->setPatrolPositions(positions);
    
    
    return patrol->getCmd(t);
  }
  
protected:
  SPatrol* patrol;
};



class BStuttgartGoto:public BDIBehavior {
  public:
    SPhysGotoPosAvoidObstacles  *gotopos ;
    Time lastmessage;
    
    BStuttgartGoto() throw() : BDIBehavior("BStuttgartGoto") {
	 gotopos= new SPhysGotoPosAvoidObstacles();
	}

    bool checkInvocationCondition(const Time& time) throw() {
  
      if (stuttpointer->womodata.gotgoto)
      {
	gotopos->init(Vec(stuttpointer->womodata.gotox,stuttpointer->womodata.gotoy),Angle::deg_angle(stuttpointer->womodata.theta),true);
	stuttpointer->womodata.gotgoto=false;
	lastmessage=time;
	return true;
      }
    
      return false;
   	}

bool checkCommitmentCondition(const Time& time) throw() {
    if (stuttpointer->womodata.gotgoto)
      {
	gotopos->init(Vec(stuttpointer->womodata.gotox,stuttpointer->womodata.gotoy),Angle::deg_angle(stuttpointer->womodata.theta),true);
	stuttpointer->womodata.gotgoto=false;
	lastmessage=time;
	return true;
      }
      if (abs(time.diff_msec(lastmessage))>1000) return false;
      
  }

virtual DriveVector getCmd(const Time& t) throw(TribotsException) {

 return gotopos->getCmd(t);
  
  
  
    }   
    
  
  
};




class BStuttgartHomeNoball : public BDIBehavior {
public:
	SPhysGotoPosAvoidObstacles *gotopos;
  
		// last point in time before release of the ball
		Time freePlayTime;
		
		
	BStuttgartHomeNoball() throw() : BDIBehavior("BStuttgartHomeNoball") {
	 gotopos= new SPhysGotoPosAvoidObstacles();
  
	}



	bool checkInvocationCondition(const Time& time) throw() {
  
return true; 
	}

bool checkCommitmentCondition(const Time& time) throw() {
		return checkInvocationCondition(time);
	}

virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
    const FieldGeometry& field = MWM.get_field_geometry();      
      
    

  gotopos->init(Vec(+1000,-4500),Angle::deg_angle(15),true);

  return gotopos->getCmd(t);
 }

};

class BStuttgartStandardSituation : public BDIBehavior {
public:
	SPatrol *patrol;
	SPhysGotoPosAvoidObstacles *gotopos;
        BBlockWayToGoal* blockGoal;
		// last point in time before release of the ball
		Time freePlayTime;
	bool waitsomemore;
 	Time last_ref_time;	
		
	BStuttgartStandardSituation() throw() : BDIBehavior("BStuttgartStandardSituation") {
	 patrol=new SPatrol();
	 gotopos= new SPhysGotoPosAvoidObstacles();
	 blockGoal= new BBlockWayToGoal(2000,true);// addOption(new BPreOwnThrowIn());
	 // addOption(new BPreOwnKickOff());
	//  addOption (new BBlockWayToGoal(2000,true));
  
	}



	bool checkInvocationCondition(const Time& time) throw() {
  
bool inStandard=((
     REFSTATE == preOpponentThrowIn ||REFSTATE == preOpponentGoalKick || REFSTATE == preOpponentCornerKick ||REFSTATE == preOpponentFreeKick ||REFSTATE == preOpponentKickOff ||REFSTATE == preDroppedBall ||
     REFSTATE == preOwnThrowIn ||REFSTATE == preOwnGoalKick ||REFSTATE == preOwnFreeKick ||REFSTATE == preOwnKickOff ||REFSTATE == preOwnCornerKick ||
     REFSTATE == postOpponentThrowIn ||REFSTATE == postOpponentGoalKick ||REFSTATE == postOpponentCornerKick ||REFSTATE == postOpponentFreeKick ||REFSTATE == postOpponentKickOff
     )
     );

if (inStandard==true&& (REFSTATE==preOpponentKickOff||REFSTATE==preOpponentGoalKick ||REFSTATE==preOpponentFreeKick ||REFSTATE==preOpponentThrowIn||REFSTATE==preOpponentCornerKick ))  // in diesen Situationen muss nach dem anpfiff noch gewartet werden """
waitsomemore=true;
else 
waitsomemore=false;

last_ref_time=time;


return inStandard;


	}

bool checkCommitmentCondition(const Time& time) throw() {

		if ((REFSTATE== freePlay )
			&& fabs(time.diff_sec(last_ref_time))<10 
			&& waitsomemore)  return true; 
		else 
	return checkInvocationCondition(time);


	}

virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
    const FieldGeometry& field = MWM.get_field_geometry();      
      
  
    gotopos->init(Vec(0,-5500),Angle::deg_angle(0),true);

  
  
  
if (REFSTATE==preOwnThrowIn)gotopos->init(Vec(+2000,-3500),Angle::deg_angle(+30),true);

if (REFSTATE==preOwnKickOff)gotopos->init(Vec(+00,-6000),Angle::deg_angle(+0),true);
if (REFSTATE==preOpponentKickOff)gotopos->init(Vec(+100,-6000),Angle::deg_angle(+0),true);


if (REFSTATE==preOwnThrowIn)gotopos->init(Vec(+2000,-3500),Angle::deg_angle(+30),true);

if (REFSTATE==preOpponentThrowIn)return blockGoal->getCmd(t);
if (REFSTATE==preOwnThrowIn)return blockGoal->getCmd(t);
if (REFSTATE==preOwnGoalKick)return blockGoal->getCmd(t);
if (REFSTATE==preOpponentGoalKick)return blockGoal->getCmd(t);
if (REFSTATE==preOwnFreeKick)return blockGoal->getCmd(t);
if (REFSTATE==preOpponentFreeKick)return blockGoal->getCmd(t);
if (REFSTATE==preOwnCornerKick)return blockGoal->getCmd(t);
if (REFSTATE==preOpponentCornerKick)return blockGoal->getCmd(t);

  
  
  return gotopos->getCmd(t);
  
  
  
    }       
};








// Ballstack - geht immer an, wenn der Spieler zum Ball darf
//
class BallOwnerStackStuttgart : public BDIBehavior {
private:
  int counter;          ///< zur Ueberbrueckung weniger Zyklen ohne Ballbesitz
  bool obeyPenaltyAreaRules;///< An die nur-1-Spieler-im-Strafraum Regel halten?

  
public:
  /**
   * Konstruktor des BallStacks.
   *
   * \param leftActionArea Aktivitaetsbereich des linken Verteidigers
   * \param rightActionArea Aktivitaetsbereich des rechten Verteidigers
   * \param opponent_type Typ des Angreifers. Vorlaeufige Methode eine 
   *                      Strategie zu spezifizieren.
   */
  BallOwnerStackStuttgart(                 int longPassKickDuration,
                 int hackKickLength,
                 bool obeyPenaltyAreaRules=true) 
    : BDIBehavior("FieldPlayerStuttgartBallOwnerStack"), 
      obeyPenaltyAreaRules(obeyPenaltyAreaRules)
  {
   counter=0;
    addOption (new BShootEmergency(hackKickLength));
    //addOption (new BFarShoot(.5));
    addOption (new BDraufhalten(1.0f,45,7500,0));
    addOption (new BShoot(7500));
    //addOption (new BShootImmediately());
    //addOption (new BBefreiungsschlag());
    addOption (new BStuckDistanceShooter());
   // addOption (new BEigenMove());
   // addOption (new BBoostBallToGoal());
    addOption (new BDribbleBallStraightToGoalEvadeSidewards());


    addOption (new BDribbleBallToGoal());

   
 addOption (new BComplexApproachBallFreePlay());
  }
  
  ~BallOwnerStackStuttgart() throw () {
  }

  /**
   * Generiert einen DriveVector. Verwendet dazu die unveraenderte Methode
   * des BDI-Verhaltens (siehe BDIBehavior), ueberprueft aber vorher, ob
   * beim Teamcontrol die Rolle "ball" aktiv beantragt werden soll.
   */
  virtual DriveVector getCmd(const Tribots::Time& t) throw (Tribots::TribotsException) { 
    // Anfrage an das teamcontrol, Rolle "ball" zu bekommen.
    
    DriveVector app;
    app=BDIBehavior::getCmd(t);
  
   /* RobotLocation loc=MWM.get_robot_location(t);
    if (loc.pos.y>-1000)
    {
	Vec back=loc.pos-Vec(loc.pos.x,-1000);
        back=back*0.001f;
        back=back.rotate(-loc.heading);    
      	 
	 app.vtrans=app.vtrans-back;
    
    }
    */
    
    
    return app;
  
  
  }

  virtual void cycleCallBack(const Time& t) throw() {
    BDIBehavior::cycleCallBack(t);
    WBOARD->checkMessageBoard();
    unsigned int robotSelfId = MWM.get_robot_id(); 
  }



  virtual bool checkCommitmentCondition(const Time& t) throw() {
    
    
    // kein ballbesitz!
    
    return checkInvocationCondition(t);
    
    
    }                // mal den ball gehabt

  virtual bool checkInvocationCondition(const Time& t) throw() {
      stuttpointer->womodata.ihavetheball=false;
    if (WBOARD->doPossessBall(t)) { // bei Ballbesitz weiter machen...
      counter = 30;  // 30 Zyklen weitermachen, auch wenn ballbesitz weg ist
    }
  
   if (counter > 0) {
      stuttpointer->womodata.ihavetheball=true;
      LOUT << "BallStack: Den " << 31-counter << ". Zyklus keinen Ballbesitz "
	   << "mehr." << endl;
      --counter;
      }
  

   const BallLocation& ball = MWM.get_ball_location(t);
    const RobotLocation& pos = MWM.get_robot_location(t);
  
   if (stuttpointer->womodata.stuttgarthasball) return false;    
   if (ball.pos_known == BallLocation::unknown) return false;
   if (ball.pos_known == BallLocation::raised)  return false;
    if ((ball.pos-pos.pos).length()>2500) return false;
    if (REFSTATE!=freePlay) return false;
            
   return true;
  
  }  

  virtual void loseControl(const Time&) throw(TribotsException) 
  { counter = 0; }
};   


static const char *_tribots_stuttgart_roles[5] = { "ballL", "ballR", "left", "right", "safety" };

bool
HumanCoopPlayer::set_role(const char* role) throw ()
{
  if (BehaviorPlayer::set_role(role)) {
    FieldPlayerStuttgart_role = std::string(role);      // fuer "interne" behaviors
    WBOARD->setZonePressureRole(std::string(role));
    return true;
  }
  return false;
}

class BFPStuttgartUpdate : public Behavior {
public:
  
 // BBClient * bbc;
 StuttgartInterface * stutt;


  BFPStuttgartUpdate() : Behavior("BFPStuttgartUpdate")
  {
	//bbc=new BBClient();
	//bbc->init(string("localhost"));
	cout <<" setting up stuttgartinterface"<<endl;
	stutt=new StuttgartInterface();
    
	stuttpointer=stutt;
	
	stutt->hostname="localhost";    
	stutt->port=20002;    
	stutt->port=CONFIG_GETVAR("stuttgart.cfg","mdp3serverport",20002,"mdp3serverport",0,100000);
	stutt->hostname=CONFIG_GETVAR3("stuttgart.cfg","mdp3servername","localhost","mdp3servername");
	
	
}
  
  void updateTactics (const TacticsBoard& tb) throw () {
  }

  virtual bool checkInvocationCondition(const Time& t) throw() {
    return false;
  }
  
  virtual void cycleCallBack(const Time& t) throw() {
    

  //    string sendmessage;

  //    bbc.bbcomm->send (sendmessage.c_str ());
     // bbc->update ();
     
     
     //cout <<"communication!!! "<<endl;
	stutt->communicate();

  //MWM.update_refbox (SIGstart); 
//cout <<"Refboxcommand:"<< stutt->womodata.refboxcommand<<endl;

Tribots::RefboxSignal refsig;
refsig=SIGnop;
if (stutt->womodata.refboxcommand=="go")refsig=SIGstart;
else if (stutt->womodata.refboxcommand=="gameInterrupt")refsig=SIGstop;
else if (stutt->womodata.refboxcommand=="ownKickOff")refsig=SIGownKickOff;
else if (stutt->womodata.refboxcommand=="goalKickOwn")refsig=SIGownGoalKick;
else if (stutt->womodata.refboxcommand=="cornerKickOwn")refsig=SIGownCornerKick;
else if (stutt->womodata.refboxcommand=="throwInOwn")refsig=SIGownThrowIn;
else if (stutt->womodata.refboxcommand=="throwInOwn")refsig=SIGownThrowIn;
else if (stutt->womodata.refboxcommand=="freeKickOwn")refsig=SIGownFreeKick;
else if (stutt->womodata.refboxcommand=="oppKickOff")refsig=SIGopponentKickOff;
else if (stutt->womodata.refboxcommand=="goalKickOpp")refsig=SIGopponentGoalKick;
else if (stutt->womodata.refboxcommand=="cornerKickOpp")refsig=SIGopponentCornerKick;
else if (stutt->womodata.refboxcommand=="throwInOpp")refsig=SIGopponentThrowIn;
else if (stutt->womodata.refboxcommand=="freeKickOpp")refsig=SIGopponentFreeKick;
else if (stutt->womodata.refboxcommand=="stop")refsig=SIGstop;
else if (stutt->womodata.refboxcommand=="penaltyKickOwn")refsig=SIGownPenalty;
else if (stutt->womodata.refboxcommand=="penaltyKickOpp")refsig=SIGopponentPenalty;
else if (stutt->womodata.refboxcommand=="droppedBall")refsig=SIGdroppedBall;
else if (stutt->womodata.refboxcommand=="ownGoal")refsig=SIGownGoalScored;
else if (stutt->womodata.refboxcommand=="oppGoal")refsig=SIGopponentGoalScored;
else if (stutt->womodata.refboxcommand=="")refsig=SIGnop;
else if (stutt->womodata.refboxcommand=="restartEngine")refsig=SIGnop;  // stuttgart irrelevant
else {LOUT <<"UNHANDLED REFEREE SIGNAL "<< stutt->womodata.refboxcommand<<"]"<<endl; ;}

  

  MWM.update_refbox (refsig); 

  }
  
  virtual DriveVector getCmd(const Tribots::Time&) 
    throw (Tribots::TribotsException) {
    DriveVector dv; 
    dv.kick = 0;
    dv.vrot = 0;
    dv.vtrans = Vec(0.,0.);
    return dv;
  }


};


class VirtObstacles : public FataMorgana {
  virtual void update(WorldModelTypeBase* wm) throw() {
/*    LOUT << "Virtual obstacles added" << endl;
    wm->add_obstacle_absolute(Vec(0, 1000.), 500.);
    wm->add_obstacle_absolute(Vec(-500, 3000.), 500);
    wm->add_obstacle_absolute(Vec(700, 2000.), 500);
*/
  }
};




} // Ende unbenannter Namespace

HumanCoopPlayer::FieldPlayerStuttgart (const ConfigReader& cfg) throw () 
  : BehaviorPlayer ("FieldPlayerStuttgart", _tribots_stuttgart_roles, 5) 
{
//  MWM.add_fata_morgana(new VirtObstacles());
    
  set_role("ballL");  // anfaengliche Rolle setzen
    
  // Das WhiteBoard veranlassen, die Configs auzuwerten ///////////////////////
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Area, die der Roboter nicht verlassen darf festlegen (Training) //////////
  // ACHTUNG: Am 23.4.08 auf kleinere Seitenbanden eingestellt, da zu oft in Banden gefahren.
  XYRectangle area(Vec( fgeom.field_width /2. + 0., 
                        fgeom.field_length/2. + 0.),
                   Vec(-fgeom.field_width /2. - 0.,
                       -fgeom.field_length/2. - 0)); 
  
  // Aktivitaetsbereiche fuer Rollen festlegen ////////////////////////////////
  Vec target(0., fgeom.field_length/4.);

  // Kickdauern und Passwahrscheinlichkeiten einlesen /////////////////////////
  int longPassKickDuration = 40;
  int shortPassKickDuration = 20;
  int throwInKickDuration = 7; // nur fuer roboter mit harting kicker relevant
  int shotKickDuration = 34; // nur fuer roboter mit harting kicker relevant // GESCHWINDIGKEITSHACK
  double standardPassProbability = 0.75;
  double standardDribbling = 0.25;
  int standardPositioning = 0;
  
  // Optionsstack fuellen /////////////////////////////////////////////////////
  addOption (new BGameStopped());
  addOption (new BLeaveGoal());
  addOption (new BStuttgartStandardSituation());  
  addOption (new BallOwnerStackStuttgart(longPassKickDuration,shotKickDuration));   // call by a teammate
  addOption (new BStuttgartGoto());  // Dieses Behavior geht an, wenn eine message erhalten wurde
  addOption (new BBlockWayToGoal(7000));
  addOption (new BSafety());
  addOption (new BProtectGoal());
  addOption (new BStuttgartHomeNoball());
  addOption (new BEmergencyStop());
  addOption (new BFPStuttgartUpdate());
}
HumanCoopPlayer::~FieldPlayerStuttgart () throw () 
{}
