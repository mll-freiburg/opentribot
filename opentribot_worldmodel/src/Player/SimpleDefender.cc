#include <cmath>
#include "SimpleDefender.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Fundamental/geometry.h"
#include "../WorldModel/WorldModel.h"
#include "../WorldModel/FataMorgana.h"
#include "../Behavior/Skills/WithoutBall/SPatrol.h"
#include "../Behavior/SPBehavior.h"
#include "../Behavior/Behaviors/BallHandling/BStuckDistanceShooter.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/SpecialGameStates/BTestStateStop.h"
#include "../Behavior/Behaviors/WithoutBall/BLeaveGoal.h"
#include "../Behavior/Behaviors/ZonePressure/BSupportDoubleTeamMiddle.h"
#include "../Behavior/Behaviors/ZonePressure/BProtectGoal.h"

// Innerhalb dieser Datei wird die spielertypspezifische Strategie des 
// SimpleDefenders festgelegt. Die vom einfachen Verteidiger verwendeten Behaviors und
// Skills sollten soweit wie moelich generisch gehalten werden, d.h. 
// Strategiespezifische Einstellungen wie Aktionsbereiche und 
// Rollenspezifikationen sollten aussschliesslich hier ueber das setzen von 
// allgemeinen Parametern der Behaviors und durch Ableiten und ueerschreiben 
// ihrer Aktivierungsbedingungen vorgenommen werden.

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("SimpleDefender"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, 
			    PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new SimpleDefender (reader);
    }
  };
  Builder the_builder;
}

namespace Tribots {   // unbenannter Namespace notwenig, um Verwechslungen beim Linken mit SimpleDefender zu vermeiden

static string SimpleDefender_role = "line";

  class BProtectGoalConditionedSD : public BProtectGoal {
  private:
    bool isActive;
  public:
    BProtectGoalConditionedSD() : BProtectGoal(true, "BProtectGoalConditionedSD"), isActive(false)
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      if (WBOARD->getZonePressureRole() != "left" &&
          WBOARD->getZonePressureRole() != "right") {
        return false;
      }
      return
      ((WBOARD->getZonePressureRole() == "left" &&      // Hysterese: erst abschalten, wenn
        ballLocation.pos.x < 500) ||                    // ball deutlich aus der mitte raus
       (WBOARD->getZonePressureRole() == "right" &&
        ballLocation.pos.x > -500)) &&
      BProtectGoal::checkCommitmentCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      if (WBOARD->getZonePressureRole() != "left" &&
          WBOARD->getZonePressureRole() != "right") {
        return false;
      }
      return
      ((WBOARD->getZonePressureRole() == "left" &&      // ball muss auf der Seite des spielers sein, sonst
        ballLocation.pos.x < 0) ||                      // wird supportMiddle verwendet
       (WBOARD->getZonePressureRole() == "right" &&
        ballLocation.pos.x > 0))   &&
      BProtectGoal::checkInvocationCondition(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException)
    {
      const FieldGeometry& field = MWM.get_field_geometry();
      if (WBOARD->getZonePressureRole() == "left") {
        protectPos = Vec(-0.85*field.goal_width/2.,-field.field_length/2.);
      }
      else { // "right"
        protectPos = Vec(+0.85*field.goal_width/2.,-field.field_length/2.);
      }
      return BProtectGoal::getCmd(t);
    }
  };

  
  class BSupportDoubleTeamMiddleConditionedSD : public BSupportDoubleTeamMiddle {
  public:
    BSupportDoubleTeamMiddleConditionedSD() : BSupportDoubleTeamMiddle(true, "BSupportDoubleTeamMiddleConditionedSD")
    {}
    bool checkCommitmentCondition(const Time& t) throw() {
      return BSupportDoubleTeamMiddleConditionedSD::checkInvocationCondition(t);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return 
      (WBOARD->getZonePressureRole() == "left" ||
       WBOARD->getZonePressureRole() == "right") &&
      BSupportDoubleTeamMiddle::checkInvocationCondition(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException)
    {
      const FieldGeometry& field = MWM.get_field_geometry();
      if (WBOARD->getZonePressureRole() == "left") {
        protectPos = Vec(-0.75*field.goal_width/2.,-field.field_length/2.);
      }
      else if (WBOARD->getZonePressureRole() == "right") {
        protectPos = Vec(+0.75*field.goal_width/2.,-field.field_length/2.);
      }
      else {
        protectPos = Vec(0, -field.field_length/2.);
      }
      return BSupportDoubleTeamMiddle::getCmd(t);
    }
  };
  
  class BProtectLine : public Behavior {
  public:
    BProtectLine(string name="BProtectLine") 
      : Behavior(name), skill(new SGoToPosEvadeObstaclesOld()),
        headingController(4., 0.0000 , 0.00, 3., -3.)
    {
      transVel=2.5;
    }
  
    ~BProtectLine() throw ()
    {
      delete skill;
    }
    
    void cycleCallBack(const Time& t) throw ()
    {
      if (lastState != MWM.get_game_state().refstate  &&
          MWM.get_game_state().refstate == freePlay) {
        y = MWM.get_robot_location(t).pos.y;
      }
      lastState = MWM.get_game_state().refstate;
    }
  
    void updateTactics (const TacticsBoard& tb) throw () 
    {
      // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
      if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVel=1.4;
      } else if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
        transVel=1.0;
      } else if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
        transVel=3.0;
      } else { // =normal
        transVel=2.5;
      }
    } 
  
    DriveVector getCmd(const Time& t) throw(TribotsException)
    {
      const BallLocation& ballLocation = MWM.get_ball_location(t);
      const FieldGeometry& field = MWM.get_field_geometry(); 
      const RobotLocation& robot = MWM.get_robot_location(t);
      DriveVector dv;
      dv.kick=0;
    
      Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
      vector<Vec> result = intersect(LineSegment(ballLocation.pos.toVec(), Vec(0,-field.field_length/2.)),
                                     LineSegment(Vec(-field.field_width/2., y),
                                                 Vec(+field.field_width/2., y)));
      if (result.size() == 0) {  // Kann stehen bleiben: Entweder Ball hinter Verteidiger oder Ziel waere im Aus.
        dv.vtrans = Vec(0,0);
        dv.vrot = 0;
        return dv;
      } 
      Vec pongLocation = result[0];
      Vec orientation = ballLocation.pos.toVec()-robot.pos;
    
      // check, if position is free, choose position behind and to middle, 
      // if not.
      bool posFree = true;
      const ObstacleLocation obstacles = MWM.get_obstacle_location(t);
      for (unsigned int i=0; i < obstacles.size(); i++) {
        if ((obstacles[i].pos-pongLocation).length() < 500.) {
          posFree = false;
          break;
        }
      }
    
      //hindernis ausweichen
      Vec tempLocation;
      if (!posFree) {
        if (ballLocation.pos.x > 300.) {
          tempLocation = pongLocation + Vec(200., 200);  // rechts vor dem anderen
        }
        else if (ballLocation.pos.x < -300.) {
          tempLocation = pongLocation + Vec(-200., 200);   // links vo dem anderen
        }
        else { 
          if (robot.pos.x > 0) {             // in diesem Bereich ist ev. bei beiden Robotern dieses Verhalten an! deshalb da bleiben, wo der roboter ist
            tempLocation = pongLocation + Vec(300., 200.);
          }
          else {
            tempLocation = pongLocation + Vec(-300., -200.);
          }
        }
        pongLocation = tempLocation;
      }
    
      
      if ((pongLocation - robot.pos).length() < 100) {
        skill->init(robot.pos, transVel, orientation);
      } else {
	      skill->init(pongLocation, transVel, orientation);
      }
    
      return skill->getCmd(t);  
    }
  
    bool checkInvocationCondition(const Time& tt) throw()
    {
      bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
      bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
    
      return (seheBall || seheKommuniziertenBall) && SimpleDefender_role == "line" ;
    }
  
    bool checkCommitmentCondition(const Time& tt) throw()
    {
      return BProtectLine::checkInvocationCondition(tt);
    }
  protected:
    SKILL* skill;
    PIDController headingController;
    double transVel;
    double y;
    int lastState;
  };
  
  
  
//
// BPatrol verwendet den Skill SPatrol mit geeigneten Patrolpositionen,
// die von der aktuellen Rolle abgeleitet werden.
//
class BPatrolSD : public Behavior {
public:
 
  BPatrolSD() : Behavior("BPatrolSD"), patrol(new SPatrol()) {};
  ~BPatrolSD() throw() { delete patrol; }
  
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);

    return (ball.pos_known == BallLocation::unknown); 
  }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return BPatrolSD::checkCommitmentCondition(t);
  }
  virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
    const FieldGeometry& field = MWM.get_field_geometry();      
    const RobotLocation& robot = MWM.get_robot_location(t);
      
    std::vector<Vec> positions;

    if (SimpleDefender_role == "left") {
      patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
      patrol->setPatrolPositions(Vec(-field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(-field.penalty_area_width/2, -1000.));
    }
    else if (SimpleDefender_role == "right") {
      patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
      patrol->setPatrolPositions(Vec(field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(field.penalty_area_width/2, -1000.));
    }
    else {
      positions.push_back(Vec(-field.field_width/4.,robot.pos.y));
      positions.push_back(Vec(+field.field_width/4.,robot.pos.y));
      patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
      patrol->setPatrolPositions(positions);
    }
    return patrol->getCmd(t);
  }
  
protected:
  SPatrol* patrol;
};


  
  
  class BUseKicker : public Behavior {
  public:
    
    BUseKicker() : Behavior("BUseKicker") {};
    
    virtual bool checkCommitmentCondition(const Time& t) throw() {
      return false;
    }
    virtual bool checkInvocationCondition(const Time& t) throw() {
      return WBOARD->doPossessBall(t);
    }
    virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
      DriveVector dv;
      dv.kick = true;
      return dv;
    }
  };

static const char *_tribots_simple_defender_roles[3] = { "line", "left", "right" };

bool
SimpleDefender::set_role(const char* role) throw ()
{
  if (MultiRolePlayer::set_role(role)) {
    SimpleDefender_role = std::string(role);      // fuer "interne" behaviors
    WBOARD->setZonePressureRole(std::string(role));
    return true;
  }
  return false;
}


SimpleDefender::SimpleDefender (const ConfigReader& cfg) throw () 
  : BehaviorPlayer ("SimpleDefender", _tribots_simple_defender_roles, 3) 
{
    
  set_role("line");  // anfaengliche Rolle setzen
    
  // Das WhiteBoard veranlassen, die Configs auzuwerten ///////////////////////
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Area, die der Roboter nicht verlassen darf festlegen (Training) //////////
  // ACHTUNG: Am 23.4.08 auf kleinere Seitenbanden eingestellt, da zu oft in Banden gefahren.
  XYRectangle area(Vec( fgeom.field_width /2. + 500., 
                        fgeom.field_length/2. + 500.),
                   Vec(-fgeom.field_width /2. - 500.,
                       -fgeom.field_length/2. - 500)); 
  
  // Aktivitaetsbereiche fuer Rollen festlegen ////////////////////////////////
  Vec target(0., fgeom.field_length/4.);

  XYRectangle ballLeftActiveArea(Vec(-fgeom.field_width   /2. - 2000., 
                                     fgeom.field_length   /2. + 2000.), 
                                 Vec(fgeom.field_width    /2. -  500.,
                                     -fgeom.field_length  /2. - 2000.));
  XYRectangle ballRightActiveArea(Vec(fgeom.field_width   /2. + 2000., 
                                      fgeom.field_length  /2. + 2000.),
                                  Vec(-fgeom.field_width  /2. +  500.,
                                      -fgeom.field_length /2. - 2000.));
  
  // von gegn. grundlinie bis 2000.mm hinter grundlinie, von 2000.mm ausserhalb
  // der Seitenlinie bis 750.mm ber die tor-tor-linie (1,5m ueberlappung)
  XYRectangle leftActiveArea(Vec(-fgeom.field_width/2. - 2000., 
                                 fgeom.field_length/2.), 
                             Vec(3250., 
                                 -fgeom.field_length/2.-2000.));
  XYRectangle rightActiveArea(Vec(fgeom.field_width/2. + 2000., 
                                  fgeom.field_length/2),
                              Vec(-3250., 
                                  -fgeom.field_length/2.-2000.));
  // Bis etwas vor dem Torraum, nicht bis ganz zur seitenlinie
  XYRectangle safetyActiveArea(Vec(-fgeom.penalty_area_width/2.-1500., 
                                   -fgeom.field_length/2.-1000.),  // nicht sonderlich weit rausfahren
                               Vec(+fgeom.penalty_area_width/2.+1500., 
                                   -fgeom.field_length/2.+fgeom.penalty_area_length+2000.));

  // Optionsstack fuellen /////////////////////////////////////////////////////
  
  addOption (new BGameStopped());
  addOption (new BTestStateStop());
  addOption (new BUseKicker());
  addOption (new BLeaveGoal());
  addOption (new BStayInsideArea(area, target)); // Training
  addOption (new BProtectLine());
  addOption (new BProtectGoalConditionedSD());  // Spieler 2. Reihe (wenn nicht Support), kurzen Pfosten decken
  addOption (new BSupportDoubleTeamMiddleConditionedSD());    // Spieler 2. Reihe Ballgegenseite
  addOption (new BPatrolSD());
  addOption (new BEmergencyStop());
}

SimpleDefender::~SimpleDefender () throw () 
{}
  
}
