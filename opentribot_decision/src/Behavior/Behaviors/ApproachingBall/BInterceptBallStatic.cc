#include "../../../Structures/Journal.h"
#include "BInterceptBallStatic.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>
#include <fstream>
#include <stdexcept>

using namespace Tribots;
using namespace std;
const double BInterceptBallStatic::MAX_ROTATE_SPEED=1.5*M_PI; // really the max
const double BInterceptBallStatic::MAX_INTERSECT_SPEED=3; 

BInterceptBallStatic::BInterceptBallStatic () throw () 
  : Behavior ("BInterceptBallStatic"), isActive(false)
{

  // initialise hysteresis
  cycle = 0;
  for (unsigned int i=0; i<LOOKAHEAD; i++) { 
     invocableCycle[i] = false; 
  }

  // set up vertical movement function
  // = ball catching
  breakF.addVertex(0.05, .25);
  breakF.addVertex(.6, .4);
  breakF.addVertex(1.2, 1.);
  breakF.addVertex(100., 1.);

  goToBall.addVertex(1., 1.);      // schneller Anstieg
  goToBall.addVertex(100., 1.);      // schneller Anstieg

  // set up rotational movement
  rotateFactor.addVertex(.0,.0);     // Beginn
  rotateFactor.addVertex(.05,.2);    // langsamer Anstieg
  rotateFactor.addVertex(.2,1.0);    // super schnelle Rotation
  rotateFactor.addVertex(1,1);       // Endpunkt

  // set up movement to the intersection point
  intersectSpeed.addVertex(0.0, 0.0);
  intersectSpeed.addVertex(1.0, 1.0);
  intersectSpeed.addVertex(100.0, 1.0);
}


void BInterceptBallStatic::cycleCallBack (const Time &t) throw () {
   // count possible activations
   // if treshold reached then intercept
   cycle = (cycle + 1) % LOOKAHEAD;
   LOUT << "BInterceptBallStatic Condition: "  << (invocableCycle[cycle] = checkConditions(t)) << endl; 
}



bool BInterceptBallStatic::checkConditions (const Time& texec) throw () {
   const RobotLocation& robot_exec (MWM.get_robot_location (texec));
   const BallLocation& ball_exec (MWM.get_ball_location (texec));
 
   /* dont go off, we want to break the ball
   if (WBOARD->doPossessBall(texec)) {
      LOUT << "do possess ball\n ";
      return false;
   }*/
   if (ball_exec.pos_known == BallLocation::unknown) {
      LOUT << "BInterceptBallStatic: ball_exec.pos_known == BallLocation::unknown\n" ;
	   return false;
   }
  
   // only if the ball is faster than zero to get a good trajectory of the ball movement
   // and not to get misleaded by irregular vision errors
  if(ball_exec.velocity.toVec().length() < (isActive ? .3f : 1.f)) {
      LOUT << "BInterceptBallStatic: ball speed to low. Should be above " << (isActive ? .3f : 1.f) << "\n" ;
      return false;
  }

  // go off, if ball is to far away and to slow
  double balldist = (ball_exec.pos.toVec()-robot_exec.pos).length()/1000;
  double impacttime = balldist / ball_exec.velocity.toVec().length();
  if (impacttime > 2) {
      LOUT << "BInterceptBallStatic: ball to far away and speed to low. Impacttime is " << impacttime << ", but should be below 2!" << "\n" ;
      return false;
  }


 // if(fabs((robot_exec.pos-ball_exec.pos.toVec()).length())<= 1500) {
 //    LOUT << "BInterceptBallStatic: not intercepting, since distance to the ball below 1.5m .\n ";
 //    return false;
 // }

   //Geht nur an wenn der Winkel zwischen Ballbewegung und Linie zum Ball kleiner als 30 Grad ist
 /*  if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180())
         >
       (1 / ((robot_exec.pos-ball_exec.pos.toVec()).length())+1)*30) 
   {*/
  Vec relBall = WBOARD->getAbs2RelFrame(texec) * ball_exec.pos.toVec();
  if( (fabs((robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180())>45) &&
      !(fabs(relBall.x) < 500. && relBall.y > 0 && relBall.y < 700.)){
    LOUT << "BInterceptBallStatic-condition not fulfilled, because ball not in box "
            "and the angle to the ballmovement bigger than 45 deg: ";
    LOUT << fabs((robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180()) << endl;
    return false;
  }
  if ((WBOARD->getAbs2RelFrame(texec) * ball_exec.pos).y < 0) {
    LOUT << "BInterceptBallStatic: ball is behind me." << endl;
    return false;
  }
  
  return true;
}


bool BInterceptBallStatic::checkInvocationCondition (const Time& texec) throw () {
   // special case: if speed high enough then active
   // usual hysteresis-based invocation checking
   bool invocable=true;
   for (unsigned int i=0; i<LOOKAHEAD; i++) { 
      invocable = invocable&&invocableCycle[i]; 
   }
   return invocable;
}


bool BInterceptBallStatic::checkCommitmentCondition (const Time& texec) throw () {
  bool invocable=false;
  for (unsigned int i=0; i<LOOKAHEAD; i++) {  
    invocable = invocable || invocableCycle[i]; 
  }
  LOUT << "BInterceptStatic commitment condition: " << invocable << endl;
  return invocable;
}


DriveVector BInterceptBallStatic::getCmd(const Time& texec) throw () {
   isActive = true;
	DriveVector dest;
	const RobotLocation& robot_exec (MWM.get_robot_location (texec));
	const BallLocation& ball_exec (MWM.get_ball_location (texec));
  dest.vtrans = Vec(0.,0.);
  dest.vrot = 0.;
  dest.kick=false;
  if (ball_exec.pos_known == BallLocation::raised) {
    return dest;
  }
   try {
      Line ball_line (ball_exec.pos.toVec(), 
                      ball_exec.pos.toVec() + 
                      ball_exec.velocity.toVec());


      // activate catching
      if ((robot_exec.pos-ball_exec.pos.toVec()).length() < 1000) {
         LOUT << "BInterceptBallStatic: active catching" << endl;
         const double MAX_SPEED = 3.0;
         // old approach: good stopping but going backwards a lot
        //double speed = MAX_SPEED* 
	//		goToBall.getValue(
        //               (ball_exec.pos.toVec()-robot_exec.pos).length()/1000.) 
        //         - MAX_SPEED*(breakF.getValue(ball_exec.velocity.toVec().length()-0.5));
        //
/*
         LOUT << "approaching speed: " 
	      << goToBall.getValue(
	        (ball_exec.pos.toVec()-robot_exec.pos).length()/1000.) 
              << endl;
         LOUT << "stopping speed: " << (-MAX_SPEED)*(breakF.getValue(ball_exec.velocity.toVec().length())) << endl;
         */
         double speed = - ball_exec.velocity.toVec().length()/3;
         if (speed < -MAX_SPEED) speed = -MAX_SPEED;

         LOUT << "stopping speed: " << speed << " (ballspeed: " << ball_exec.velocity.toVec().length() << ")" << endl;

         dest.vtrans = (ball_exec.pos.toVec() - robot_exec.pos).
                        normalize() * speed * -robot_exec.heading;	
      }

      double hdiff = (robot_exec.heading - 
                     (ball_exec.velocity.toVec().angle()+
                      Angle::quarter)).get_rad_pi()/M_PI;

      dest.vrot = -(fabs(hdiff)/hdiff)  // get direction
                  *MAX_ROTATE_SPEED
                  *rotateFactor.getValue(fabs(hdiff)); // break according to rotation
      LOUT << "angular diff: " << hdiff << " -- ";
      LOUT << "rotateFactor: " << rotateFactor.getValue(fabs(hdiff)) << "\n";
 
		Line botToBallLine (robot_exec.pos, 
				              robot_exec.pos + 
                          ball_exec.velocity.toVec()*Angle::quarter);

		Vec p = intersect (botToBallLine, ball_line);
		LOUT <<"% blue solid circle "<< p <<" 400"<<endl;

      // go to intersection point
		dest.vtrans += (p-robot_exec.pos).normalize() * 
           (MAX_INTERSECT_SPEED * intersectSpeed.getValue((p-robot_exec.pos).length()/1000.))
            * -robot_exec.heading;
      LOUT << "distance from intersection point: " << ((p-robot_exec.pos).length()/1000.) << " -- ";
      LOUT << "intersect speed: " << MAX_INTERSECT_SPEED * intersectSpeed.getValue((p-robot_exec.pos).length()/1000.) << "\n";
      LOUT << "setting vtrans to " << dest.vtrans << "\n";
      LOUT << "setting vrot to " << dest.vrot << "\n";
	} catch(invalid_argument&) { // TODO: who throws this?
      LOUT << "BInterceptBallStatic: invalid argument: rollt parallel zum Spieler\n";
   } catch(exception& e) {
      JERROR("exception in BInterceptBallStatic: ");
      JERROR(e.what());
   }
	dest.kick=false;
	return dest;
}
