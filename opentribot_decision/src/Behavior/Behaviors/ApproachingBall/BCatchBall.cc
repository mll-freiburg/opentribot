//#include <fstream.h>
#include "BCatchBall.h"
#include "../../../Structures/Journal.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>
#include <fstream>
#include <stdexcept>

using namespace Tribots;
using namespace std;


BCatchBall::BCatchBall () throw () 
  : Behavior ("BCatchBall") {

//  breakF.addVertex(0.05, .25);
//  breakF.addVertex(.6, .4);
//  breakF.addVertex(1.2, 1.);
  breakF.addVertex(1., 1.);
//  breakF.addVertex(100., 1.); 
  goToBall.addVertex(1., 1.);      // schneller Anstieg
  goToBall.addVertex(100., 1.);      // schneller Anstieg
}

bool BCatchBall::checkInvocationCondition (const Time& texec) throw () {
   const RobotLocation& robot_exec (MWM.get_robot_location (texec));
   const BallLocation& ball_exec (MWM.get_ball_location (texec));

   if(ball_exec.pos_known != BallLocation::known) {
      LOUT << "I can't see the ball. \n";
	   return false;
   }
   
   if (WBOARD->doPossessBall(texec)) { // target reached !
      //WBOARD->resetPossessBall(); // trick to let BApproachBallStatic make one cycle
      return false;
   }

   //Geht nur an wenn der Winkel zwischen Ballbewegung und Linie zum Ball kleiner als 30 Grad ist
   if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180())
         >
       (1 / ((robot_exec.pos-ball_exec.pos.toVec()).length())+1)*30) 
   {
    LOUT << "BCatch aborted, because the angle to the ballmovement is too big\n";
    LOUT << "angle         : "<< fabs((robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180()) <<"\n";
    return false;
  }
  
   // TODO: the next condition should depend upon the distance to the ball to!
   // Robot has to look at the ball
   if ( fabs((robot_exec.heading + Angle::quarter - (ball_exec.pos.toVec() - robot_exec.pos).angle()).get_deg_180()) > 15 ) {
      return false;
   }      

   // if ball is to slow dont activate bcatchball
   if(ball_exec.velocity.toVec().length() <.5f) {
      return false;
   }

   // catch only if balldist is in the following range
   if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).length()) > 1500 )
   {
      LOUT << "BCatchBall: Ball is to far away -> catching makes no sense.\n";
      return false;
   }
//    ofstream myfile;
//    myfile.open ("experiment.txt",ios::out);
//    myfile << "BallSpeed , RobotSpeed";
//    myfile.close();
                    
  return true;
}

bool BCatchBall::checkCommitmentCondition (const Time& texec) throw () {
   /*
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
   if(!(ball_exec.pos_known == BallLocation::known ||
     ball_exec.pos_known == BallLocation::communicated)){
        LOUT << "I don't know the location of the ball. \n";
        return false;
     }

   if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).length()) < 400 ) 
   {
   LOUT << "Close enough. \n";
   return false;
   }
   */
  return checkInvocationCondition (texec);
}


DriveVector BCatchBall::getCmd(const Time& texec) throw () {
	DriveVector dest;
	const RobotLocation& robot_exec (MWM.get_robot_location (texec));
	const BallLocation& ball_exec (MWM.get_ball_location (texec));
   try{
      dest.vtrans = Vec(0.,0.);
	   dest.vrot = 0.;
      double speed = goToBall.getValue(
         (ball_exec.pos.toVec()-robot_exec.pos).length()/1000.) 
         - (breakF.getValue(ball_exec.velocity.toVec().length()-0.5));
    
      LOUT << " dist_length     : " 
           << (ball_exec.pos.toVec()-robot_exec.pos).length()/1000. 
           << "\n";
      LOUT << " ball.pos        : " << ball_exec.pos.toVec() << "\n";
      LOUT << " robot.pos       : " << robot_exec.pos << "\n";
      LOUT << " direction       : " 
           << ball_exec.pos.toVec()-robot_exec.pos << "\n";
      LOUT << " 	   go to ball speed: " 
           << goToBall.getValue(
             (ball_exec.pos.toVec()-robot_exec.pos).length()/1000.) 
           << "\n";
      LOUT << " ball velocity   : " 
           << ball_exec.velocity.toVec().length() << "\n";
      LOUT << "      break speed: " 
           << 0.-breakF.getValue(ball_exec.velocity.toVec().length()) 
           << "\n";
      dest.vtrans = (ball_exec.pos.toVec()-robot_exec.pos).normalize()
                    * speed * -robot_exec.heading;	
    
      Line dyna_pos_line (ball_exec.pos.toVec(),
                          ball_exec.pos.toVec()+
                          ball_exec.velocity.toVec());


      double hdiff=(robot_exec.heading -
                    (ball_exec.velocity.toVec().angle()+
                     Angle::quarter)).get_rad_pi()/M_PI;
      double max_rotate_speed=3.5;

      PiecewiseLinearFunction plf;
      plf.addVertex(.3,.4);      // schneller Anstieg
      plf.addVertex(1., 1.);      // schneller Anstieg
      dest.vrot = -(fabs(hdiff)/hdiff)*
                  max_rotate_speed*plf.getValue(fabs(hdiff));

      Line botToBallLine (robot_exec.pos,
                        robot_exec.pos + 
                        ball_exec.velocity.toVec()*Angle::quarter);

      Vec p = intersect (botToBallLine, dyna_pos_line);
      LOUT <<"% circle "<<p <<" 500"<<endl;

      //(vector (angle) to intersection point) * (speed scaling) * (robot coordinate system orientatio
      dest.vtrans += (p-robot_exec.pos).normalize() * 
           (2.5 * breakF.getValue((p-robot_exec.pos).length()/1000.)) 
           * -robot_exec.heading;
                
   } catch(invalid_argument&) {
      LOUT << "invalid argument\n";
      // rollt parallel zum Spieler
   } catch(exception& e) {
      JERROR("exception in BCatchBallStatic: ");
      JERROR(e.what());
   }
   dest.kick=false;                                      
	return dest;
}
