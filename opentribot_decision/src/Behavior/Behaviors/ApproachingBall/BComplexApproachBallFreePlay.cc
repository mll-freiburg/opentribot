#include "BComplexApproachBallFreePlay.h"
#include "BApproachBallDirectly.h"
#include "BApproachBallFromBehindPointingToGoal.h"
#include "BApproachBallFromBehindPointingAwayOwnGoal.h"
#include "BApproachBallFromBehindPointingToMiddle.h"
#include "BInterceptBall.h"
#include "BApproachBallAfterNonexecutedStandard.h"
#include "../WithoutBall/BAvoidGoalieArea.h"
#include "../../../Fundamental/geometry.h"
#include "../../../WorldModel/WorldModel.h"
#include <cmath>


#define DEBUG_APPROACH 0

using namespace std;
namespace Tribots 
{
  
class BInterceptBallConditioned : BInterceptBall
{
public:
  BInterceptBallConditioned(const Area* activeArea = 0) throw()
    : BInterceptBall()
  {
    if (activeArea) {
      this->activeArea = activeArea->clone();
    }
    else {
      const FieldGeometry& field =
        MWM.get_field_geometry();  
        
      // X: mindestens 1000 mm von seitenlinie weg
      // Y: Bis 700mm vorm gegnerischen Tor
      //    und Ball maximal 2000mm vor mittellinie
      this->activeArea = 
        new Quadrangle(Vec(-field.field_width  / 2. + 1000., 
                            field.field_length / 2. -  700.),
                        Vec(field.field_width  / 2. - 1000., 
                            -2000.)); 
    }    
  }
  
  virtual ~BInterceptBallConditioned() throw () {
      delete activeArea;
  }
      
  virtual bool checkCommitmentCondition(const Time& t) throw() {

    if (DEBUG_APPROACH) {
      LOUT<<"% dark_green thin dotted" << (*activeArea) << "\n";
    }
    return 
      BInterceptBall::checkInvocationCondition(t) &&
      activeArea->is_inside(MWM.get_ball_location(t).pos.toVec());
  }
  virtual bool checkInvocationCondition(const Time& t) throw() {

    return 
      BInterceptBall::checkInvocationCondition(t) &&
      activeArea->is_inside(MWM.get_ball_location(t).pos.toVec());
  }    
    
  protected:
    Area* activeArea;
};  
   
  
class BApproachBallDirectlyConditioned : public BApproachBallDirectly
{
  public:
    BApproachBallDirectlyConditioned(const Area* activeArea = 0) throw()
      : BApproachBallDirectly()
    {
      if (activeArea) {
        this->activeArea = activeArea->clone();
      }
      else {  // default
        const FieldGeometry& field =
          MWM.get_field_geometry();  
        
        // X: Mindestabstand von 1500mm zur Seitenlinie
        // Y: Bis 1700mm vorm gegnerischen Tor
        //    aber Ball mindestens 3500mm vor eigenem Tor
        this->activeArea = 
          new XYRectangle(Vec(-field.field_width / 2. + 1500., 
                              field.field_length / 2. - 1700.),
                         Vec(field.field_width / 2. - 1500., 
                             -field.field_length / 2. + 3500.)); 
      }
    }
    
    virtual ~BApproachBallDirectlyConditioned() throw () {
      delete activeArea;
    }
      
    virtual bool checkCommitmentCondition(const Time& t) throw() {
      if (DEBUG_APPROACH) {
        LOUT<<"% dark_green thin dotted" << (*activeArea) << "\n";
      }
      return 
        BApproachBallDirectly::checkCommitmentCondition(t) &&
        activeArea->is_inside(MWM.get_ball_location(t).pos.toVec());
    }
    virtual bool checkInvocationCondition(const Time& t) throw() {
      return 
      BApproachBallDirectly::checkInvocationCondition(t) &&
        activeArea->is_inside(MWM.get_ball_location(t).pos.toVec());
    }    
    
  protected:
    Area* activeArea;
};



class BApproachBallDirectlyAfterStandard : public BApproachBallDirectly
{
public:
  BApproachBallDirectlyAfterStandard() throw()
  : BApproachBallDirectly("BApproachBallDirectlyAfterStandard"), activate(false)
  {
    tLosSent = Time();
    tLosSent.add_msec(-30);
    lastGameState = MWM.get_game_state().refstate;
  }
  
  virtual ~BApproachBallDirectlyAfterStandard() throw () {
  }
  
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    if (DEBUG_APPROACH) {
      LOUT<<"% dark_green thin dotted" << (*activeArea) << "\n";
    }
    return 
    BApproachBallDirectly::checkCommitmentCondition(t) && t.diff_sec(tLosSent) < 5 && activate;
  }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return 
      MWM.get_ball_location(t).pos.y < 0 && // ball in eigener haelfte
      BApproachBallDirectly::checkInvocationCondition(t) && t.diff_sec(tLosSent) < 5 && activate;
  }    
  
  void cycleCallBack(const Time& t) throw() 
  {
    if (lastGameState != MWM.get_game_state().refstate &&
        (MWM.get_game_state().refstate == postOpponentThrowIn ||
         MWM.get_game_state().refstate == postOpponentCornerKick ||
         MWM.get_game_state().refstate == postOpponentFreeKick ||
         MWM.get_game_state().refstate == postOpponentKickOff)) {
      tLosSent = t;
    }
    if (lastGameState != MWM.get_game_state().refstate &&
        (lastGameState == postOpponentThrowIn ||
         lastGameState == postOpponentCornerKick ||
         lastGameState == postOpponentFreeKick ||
         lastGameState == postOpponentKickOff) &&
        MWM.get_game_state().refstate == freePlay) {
      tLosSent = t;
    }
    lastGameState = MWM.get_game_state().refstate;
  }
  
  void updateTactics (const TacticsBoard& tb) throw ()
  {
    if (tb[string("StandardSituationBallDirektAnfahren")] == string("aus")) {
      activate = false;
    }
    else {
      activate = true;
    }
  }

  
protected:
  Area* activeArea;
  Time tLosSent;
  int lastGameState;
  bool activate;
}; 



class BApproachBallFromBehindPointingAwayOwnGoalConditioned : public BApproachBallFromBehindPointingAwayOwnGoal
{
  public:
    BApproachBallFromBehindPointingAwayOwnGoalConditioned(const Area* activeArea=0) throw()
      : BApproachBallFromBehindPointingAwayOwnGoal()
    {
      if (activeArea) {
        this->activeArea = activeArea->clone();
      }
      else {  // default
        const FieldGeometry& field =
          MWM.get_field_geometry();  
        
        // X: Mindestabstand von 1000mm zur Seitenlinie
        // Y: Von eigenem Tor bis 3500mm vor eigenem Tor        
        this->activeArea = 
          new Quadrangle(Vec(-field.field_width / 2. + 1000., 
                              -field.field_length / 4.),
                          Vec(field.field_width   / 2. - 1000., 
                              -field.field_length / 2.));
      }
    }
    
    virtual ~BApproachBallFromBehindPointingAwayOwnGoalConditioned() throw () {
      delete activeArea;
    }
      
    virtual bool checkCommitmentCondition(const Time& t) throw() {
      if (DEBUG_APPROACH) {
        LOUT<<"\% dark_green thin dotted" << (*activeArea) << "\n";
      }
      return
        BApproachBallFromBehindPointingAwayOwnGoal::checkCommitmentCondition(t) &&
        activeArea->is_inside(MWM.get_ball_location(t).pos.toVec());
    }
    virtual bool checkInvocationCondition(const Time& t) throw() {
      return
        BApproachBallFromBehindPointingAwayOwnGoal::checkInvocationCondition(t) && 
        activeArea->is_inside(MWM.get_ball_location(t).pos.toVec());
    }    
    
  protected:
    Area* activeArea;
};


class BApproachBallFromBehindPointingToMiddleConditioned : public BApproachBallFromBehindPointingToMiddle
{
  public:
    BApproachBallFromBehindPointingToMiddleConditioned() throw()
      : BApproachBallFromBehindPointingToMiddle()
    {}
    
    virtual ~BApproachBallFromBehindPointingToMiddleConditioned() throw () {
    }
      
    virtual bool checkCommitmentCondition(const Time& t) throw() {
      Vec ballPos = MWM.get_ball_location(t).pos.toVec();
      const FieldGeometry& field =
	MWM.get_field_geometry();

      
      // Bereiche passend zum Eigenmove gewaehlt
      // am seitenrand: nicht mehr als 1000mm in x richtung ins spielfeld
      //                und nicht weiter asl 3500 von grundlinie
      // an der endlinie: nicht mehr als 800 innerhalb des feldes und nicht
      //                  vorm tor
      if (((fabs(ballPos.x) < field.field_width/2-1000) ||
           (ballPos.y < field.field_length/2 -3500)) &&
          ((fabs(ballPos.y) < field.field_length/2. - 1200) ||
           (fabs(ballPos.x) < field.goal_width/2.))
              ) { // test ob ausserhalb a und ausserhalb b
	return false;
      }

      return
        BApproachBallFromBehindPointingToMiddle::checkCommitmentCondition(t);
    }
    virtual bool checkInvocationCondition(const Time& t) throw() {

      Vec ballPos = MWM.get_ball_location(t).pos.toVec();
      const FieldGeometry& field =
	MWM.get_field_geometry();

      // am seitenrand: nicht mehr als 1000mm in x richtung ins spielfeld
      //                und nicht weiter asl 3500 von grundlinie
      // an der endlinie: nicht mehr als 800 innerhalb des feldes und nicht
      //                  vorm tor
      if (((fabs(ballPos.x) < field.field_width/2-1000) ||
           (ballPos.y < field.field_length/2 -3500)) &&
          ((fabs(ballPos.y) < field.field_length/2. - 800) ||
           (fabs(ballPos.x) < field.goal_width/2.))
          ) { // test ob ausserhalb a und ausserhalb b
        return false;
      }
      return
        BApproachBallFromBehindPointingToMiddle::checkInvocationCondition(t);
    }    
};



BComplexApproachBallFreePlay::BComplexApproachBallFreePlay() throw()
  : BDIBehavior("BComplexApproachBallFreePlay")
{

  addOption(new BAvoidGoalieArea());
  addOption(new BApproachBallAfterNonexecutedStandard());
  addOption(new BApproachBallDirectlyAfterStandard()); // faehrt 5 Sekunden den Ball direkt an (auch ins aus), einschaltbar ueber das Taktikboard
  //addOption(new BInterceptBallRL());
  addOption(new BInterceptBall());
  // sidelines in opponent area: approach pointing to middle
  addOption(new BApproachBallFromBehindPointingToMiddleConditioned());

  // where safe: approach ball directly and turn afterwards
  addOption(new BApproachBallDirectlyConditioned(NULL));
  
  // near goal: approach ball from middle of the field pointing outwards to
  //            the sidelines
  addOption(new BApproachBallFromBehindPointingAwayOwnGoalConditioned(NULL));
  
  // default: approach ball from behind (z.B. vor gegnerischem tor)
 addOption(new BApproachBallFromBehindPointingToGoal());
}

BComplexApproachBallFreePlay::~BComplexApproachBallFreePlay() throw()
{}

}
