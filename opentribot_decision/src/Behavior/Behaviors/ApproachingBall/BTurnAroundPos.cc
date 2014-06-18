#include "BTurnAroundPos.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/random.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"
#include <cmath>
#include <fstream>
#include "../../../Structures/Journal.h"
#include <stdexcept>

using namespace Tribots;
using namespace std;

namespace {

  inline int signum (const double& x) { return (x>0 ? 1 : (x<0 ? -1 : 0)); }

}


BTurnAroundPos::BTurnAroundPos () throw () : Behavior ("BTurnAroundPos"), headingController(0, -0.0000 , -1.00, 5.0 , -5.0) {

  //skill = 0;
  //skill = new SApproachMovingBall();
  skill = new SDribbleBallToPos();
   
  headingController.reset();

  const FieldGeometry& fg (MWM.get_field_geometry()); 
  
  pa1.x = -0.5*fg.penalty_area_width;
  pa1.y = -0.5*fg.field_length;
  pa2.x = -pa1.x;
  pa2.y = -0.5*fg.field_length+fg.penalty_area_length;


  was_active=false;
  was_moving_around_goal_post=false;
  
  goal_post_right.x = 0.5*fg.goal_width;
  goal_post_right.y = 0.5*fg.field_length;
  goal_post_left.x = -0.5*fg.goal_width;
  goal_post_left.y = 0.5*fg.field_length;
  goal_post_mid.x = 0;
  goal_post_mid.y = 0.5*fg.field_length;

  target_direction_known=false;
}


BTurnAroundPos::~BTurnAroundPos() throw () {
   if (skill) {
      delete skill;
   }
}


bool BTurnAroundPos::checkInvocationCondition (const Time& t) throw () {
/*
  const BallLocation& ball = MWM.get_ball_location (t);
  const RobotLocation& robot_exec (MWM.get_robot_location (t));
  target_direction = (ball.pos.toVec()-goal_post_mid).angle();
  Angle robot_ball = (robot_exec.pos-ball.pos.toVec()).angle();
   if(fabs((robot_exec.pos-ball.pos.toVec()).length())>1000) {
      LOUT << "BTurnAroundPos: not dog-turning, since distance to the ball more one meter.\n ";
      return false;
   }
         
   return (ball.pos_known==BallLocation::known );
   */

   // try to turn only with ball
   LOUT << "BTurnAroundPos::checkInvocationCondition: " << WBOARD->doPossessBall(t) << endl;
   return WBOARD->doPossessBall(t);
}


bool BTurnAroundPos::checkCommitmentCondition (const Time& t) throw () {
   if (!WBOARD->doPossessBall(t)) {
      return false;
   }
   return true;
}


void BTurnAroundPos::gainControl(const Time& t) throw() {
  ballpos=Vec::zero_vector;
  wait_rel_to_goal=0;
  dir=0;
}


void BTurnAroundPos::loseControl(const Time& t) throw() {}


DriveVector BTurnAroundPos::getCmd(const Time& t) throw () {
  
   const BallLocation& ball = MWM.get_ball_location (t);
   const RobotLocation& robot = MWM.get_robot_location (t);

   DriveVector dv;
   try {
      Vec ausrichtungsPunkt = goal_post_mid;
   
      Angle robotHeading = robot.heading;
      Vec ballPos = ball.pos.toVec();
      Angle targetHeading=(ausrichtungsPunkt-ballPos).angle();
                
      
      /*
      Vec dogCurveTarget = ball.pos.toVec() + DOGDISTANCE*(ball.pos.toVec()-goal_post_mid).normalize();
      Vec ausrichtungsPunkt = goal_post_mid;
      float faktor = DOGCURVEFACTOR;

      Vec robotPos = robot.pos;
      Vec ballPos = ball.pos.toVec();

      Vec ausIchDiffNorm = ausrichtungsPunkt - robotPos;
      ausIchDiffNorm = ausIchDiffNorm.normalize();

       //Vector3f ausZielDiffNorm = ausrichtungsPunkt - ziel;
       //ausZielDiffNorm[2]=0;
       //ausZielDiffNorm.Normalize();
   */
   /* TODO: catch this case
       // Achtung! Wenn sich beide gegenXberliegen, macht die Funktion Mist!!
       if (Geometrie::WinkelXY(pos,ziel,ausrichtungsPunkt) > 170) {
           // bisschen drehen, damit der Agent nicht flXchtet
           ausIchDiffNorm = Geometrie::DreheXY(ausIchDiffNorm,10.0/180.0*M_PI);
       }
   */
   /*
       Vec diff = dogCurveTarget - robotPos;
       float r = diff.length() * faktor;

       //Hundekurve sascha
       Vec intermediateTarget = dogCurveTarget - ausIchDiffNorm*r;

       //Hundekurfe Foo Fighters
       //Vec intermediateTarget = dogCurveTarget - ausZielDiffNorm*r;

       // prepare keynumber
       Angle robotHeading = robot.heading;
       Angle ballHeading=(ballPos-robotPos).angle();
       Angle targetHeading=(ausrichtungsPunkt-ballPos).angle();
       DriveVector dv;
       // Calculate an approaching vector
       double hdiff=(robotHeading - (targetHeading-Angle::quarter)).get_rad_pi()/M_PI;
       double max_rotate_speed=3.5;
       double balldiff=(robotHeading-(ballHeading-Angle::quarter)).get_rad_pi()/M_PI;
       
       // ----- vrot
       double differenzanteil = headingController.getAction(0,hdiff*M_PI,MWM.get_game_state().intended_cycle_time, LOUT);
       
       PiecewiseLinearFunction plf;
       plf.addVertex(.1,.4);      // schneller Anstieg
       plf.addVertex(.2,.5);      // schneller Anstieg
       plf.addVertex(1., 1.);      // schneller Anstieg

       dv.vrot = -(fabs(hdiff)/hdiff)*max_rotate_speed*plf.getValue(fabs(hdiff));
        
        
       //----- vtrans 
       Frame2d world2robot =  WhiteBoard::getTheWhiteBoard()->getAbs2RelFrame(t);
       Frame2d robot2world =  WhiteBoard::getTheWhiteBoard()->getRel2AbsFrame(t);

       double desiredVel = 1.2;
       double maxVel = 3.0; 
        
       Vec desiredVtrans = (world2robot*intermediateTarget).normalize() * desiredVel;
         

       if (desiredVtrans.length() > maxVel) { // cut vtrans at max speed
         desiredVtrans = desiredVtrans.normalize()*maxVel;
       }
    
       //Vec evade = 1000.*approachTarget.getValue(fabs(theta.get_rad_pi())/M_PI)*
       //             targetMovement.normalize().rotate(M_PI-theta.get_rad_pi()*0.5);
       //Vec evade = (ausrichtungsPunkt-ballPos).normalize() * 500.;
       //dogCurveTarget = ballPos + evade;

       dv.vtrans = WhiteBoard::getTheWhiteBoard()->calculateEvasiveMovement(desiredVtrans, intermediateTarget, t);
        
       LOUT << "\n% blue thin solid cross " << dogCurveTarget << " 300 "
            << "\n% red thin solid cross " << intermediateTarget
            << "\n% blue word " << Vec(0,500) + dogCurveTarget
            << " DCTGT\n" ;
     */
      
       skill->setParameters(goal_post_mid, 2.2, true);

       dv = skill->getCmd(t);

   // -----------


       dv.kick = false;
       bool owns_ball = WBOARD->doPossessBall(t);
       if (owns_ball && isKickingGood(robotHeading, targetHeading, t)) {
         dv.kick = true;
       }
   } catch(exception& e) {
      JERROR("exception in BInterceptBallStatic: ");
      JERROR(e.what());
   }

  return dv;
}


bool BTurnAroundPos::isKickingGood(Angle robotHeading, Angle targetHeading, const Time& t) {
     
  double hdiff=fabs((robotHeading - (targetHeading-Angle::quarter)).get_rad_pi()/M_PI);
  LOUT << "angle to destination: " << hdiff << "\n";
   
  return (hdiff < 0.06);
}



