
#include "BInterceptBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>

using namespace Tribots;
using namespace std;


BInterceptBall::BInterceptBall () throw () 
  : Behavior ("BInterceptBall"), 
    headingController(4., 0.0000 , 0.00, 1 , -1)
{
  breakF.addVertex(0.05, .25);
  breakF.addVertex(.6, .4);
  breakF.addVertex(1.2, 1.);
  breakF.addVertex(100., 1.);
}

bool BInterceptBall::checkInvocationCondition (const Time& texec) throw () {
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  
  if(WBOARD->doPossessBall(texec) ||
    !(ball_exec.pos_known == BallLocation::known || 
     ball_exec.pos_known == BallLocation::communicated))
	return false;
  //Geht nur an wenn der Winkel zwischen Ballbewegung und Linie zum Ball kleiner 45 Grad ist
  if(fabs((robot_exec.pos-ball_exec.pos.toVec()).angle(ball_exec.velocity.toVec()).get_deg_180())>45)
	return false;
  if(ball_exec.velocity.toVec().length() <1.0f)
	return false;
  return true;
}

bool BInterceptBall::checkCommitmentCondition (const Time& texec) throw () {
  return checkInvocationCondition (texec);
}


DriveVector BInterceptBall::getCmd(const Time& texec) throw () {
	DriveVector dest;
	const RobotLocation& robot_exec (MWM.get_robot_location (texec));
	const BallLocation& ball_exec (MWM.get_ball_location (texec));

	Line dyna_pos_line (ball_exec.pos.toVec(), 
			    ball_exec.pos.toVec()+ball_exec.velocity.toVec());

	dest.vtrans = Vec(0.,0.);
	dest.vrot = 0.;

	double hdiff=(robot_exec.heading - 
				(ball_exec.velocity.toVec().angle()+Angle::quarter)).get_rad_pi()/M_PI;
	double max_rotate_speed=3.5;

	PiecewiseLinearFunction plf;
	plf.addVertex(.3,.4);      // schneller Anstieg
	plf.addVertex(1., 1.);      // schneller Anstieg
	dest.vrot = -(fabs(hdiff)/hdiff)*max_rotate_speed*plf.getValue(fabs(hdiff));


	dest.vrot =                   // calculate correction of heading
	headingController.getAction(0,
	(robot_exec.heading -
	(ball_exec.velocity.toVec().angle()+Angle::quarter)).get_rad_pi(),
	MWM.get_game_state().intended_cycle_time, 
	LOUT);

 
	try{
		Line goalie_line (robot_exec.pos, 
				  robot_exec.pos + ball_exec.velocity.toVec()*Angle::quarter);

		Vec p = intersect (goalie_line, dyna_pos_line);
		LOUT <<"circle "<<p <<" 500"<<endl;
    
		dest.vtrans = 
				(p-robot_exec.pos).normalize() * (2.5 * breakF.getValue((p-robot_exec.pos).length()/1000.)) * -robot_exec.heading;
	} catch(invalid_argument&) {
		; // rollt parallel zum Spieler
	}
	dest.kick=false;
	return dest;
}


# if 0
DriveVector BInterceptBall::getCmd(const Time& texec) throw () {
  DriveVector dest;
  const RobotLocation& robot_exec (MWM.get_robot_location (texec));
  const BallLocation& ball_exec (MWM.get_ball_location (texec));
  Vec ball2d;
  ball2d.x=ball_exec.pos.x;
  ball2d.y=ball_exec.pos.y;
  
  Vec robot2Ball=ball2d-robot_exec.pos;
  Vec ballvel;
  ballvel.x=ball_exec.velocity.x;
  ballvel.y=ball_exec.velocity.y;
  Vec targetPos= ball2d+ballvel.normalize()*robot2Ball.length();  
  Vec goaltotarget=-Vec(0,WorldModel::get_main_world_model().get_field_geometry().field_length/2)+targetPos;
  targetPos=targetPos+goaltotarget.normalize()*150;
  // So die TargetPos ist jetzt etwas hinter dem Ball! jetzt werde ich noch checken dass die TargetPos nicht ausserhalb des Spielfeldes liegt.
  double length=WorldModel::get_main_world_model().get_field_geometry().field_length;
  double width=WorldModel::get_main_world_model().get_field_geometry().field_width;
  double wieviel;
  if (fabs(targetPos.x)>width/2+300){ //über die seite nach draussen 
	  wieviel=(fabs(width/2+300)-fabs(ball2d.x))/ fabs(ball2d.x-targetPos.x);
 	  targetPos=targetPos*wieviel;
	  //cerr<<"Wieviel "<<wieviel<<endl;
  }
   if (fabs(targetPos.y)>length/2+300){ //über die seite nach draussen 
	   wieviel=(fabs(length/2+300)-fabs(ball2d.y))/ fabs(ball2d.y-targetPos.y);
 	  targetPos=targetPos*wieviel;
	  //cerr<<"Wieviel "<<wieviel<<endl;
   }
   Vec targetHeading=ballvel.normalize()*(-1);
  LOUT <<"\n% red line "<<targetPos<<" "<<targetPos+targetHeading*500<<endl;
  Angle diffangle=targetHeading.angle(goaltotarget*(-1));
  double ddiffangle=diffangle.get_deg_180();
  
  if (fabs(ddiffangle<90)){
	  targetHeading=targetHeading.normalize()+goaltotarget.normalize()*(-1);
  }
  else{
	  targetHeading=targetHeading.normalize().rotate(Angle::Angle((ddiffangle/fabs(ddiffangle))*45));
  
  
  }
  
  LOUT <<"\n% green line "<<targetPos<<" "<<targetPos+targetHeading*500<<endl;
  
  
  
  dest.vtrans = Vec(0.,0.);
  dest.vrot = 0.;

  
  
  
  double hdiff=(robot_exec.heading - 
			  ((targetHeading*(-1)).angle()+Angle::quarter)).get_rad_pi()/M_PI;
  double max_rotate_speed=3.5;

  PiecewiseLinearFunction plf;
  plf.addVertex(.3,.4);      // schneller Anstieg
  plf.addVertex(1., 1.);      // schneller Anstieg
  dest.vrot = -(fabs(hdiff)/hdiff)*max_rotate_speed*plf.getValue(fabs(hdiff));

  /*
    dest.vrot =                   // calculate correction of heading
    headingController.getAction(0,
    (robot_exec.heading -
    (ball_exec.velocity.toVec().angle()+Angle::quarter)).get_rad_pi(),
    MWM.get_game_state().intended_cycle_time, 
    LOUT);
  */
  Vec p;
p=targetPos;
    LOUT <<"circle "<<p <<" 700"<<endl;
    
    dest.vtrans = 
      (p-robot_exec.pos).normalize() * (2.5 * breakF.getValue((p-robot_exec.pos).length()/1000.)) * -robot_exec.heading;
  dest.kick=false;
  return dest;
}

#endif 
