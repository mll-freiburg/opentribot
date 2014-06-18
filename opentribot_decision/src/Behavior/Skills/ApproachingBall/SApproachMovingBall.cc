#include "SApproachMovingBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/RemoteTune.h"
#include "../../Predicates/freeCorridor.h"



#include <iostream>
#include <cmath>

using namespace Tribots;
using namespace std;

  SApproachMovingBall::SApproachMovingBall() throw ()
    : Skill ("SApproachMovingBall"), 
      headingController(2.00, -0.0000 , 1.00, 6.0 , -6.0),
      approachTarget(),
      approachSpeeds(),
      desiredVel(maxVel), active(true)
  {
    approachFacing_exp_smoothed=Vec(0,0);
    headingController.reset();
    
  // approachTarget.addVertex(0.0, 0.0);
    approachTarget.addVertex(0.2, 0.5);
    approachTarget.addVertex(0.4, 0.65);
    approachTarget.addVertex(0.5, 0.8);
    
    approachSpeeds.addVertex(0.6, 1.0);
    approachSpeeds.addVertex(1.0, 1.2);

    lowVelMod.addVertex(.1, .3);      // schneller Anstieg

    lowVelMod.addVertex(1., 1.);      // sehr langsamer Anstieg bis 1
  
    ausweichennaheball=false;
    maxVel=3.0;  
}
  
  void 
  SApproachMovingBall::loseControl(const Time&) throw(TribotsException)
  {
    headingController.reset();
  }
  
  void SApproachMovingBall::setParameters(const Vec& targetdir,
                                          double desiredVel) 
    throw(TribotsException)
  {
    approachFacing=targetdir;
    approachDirection=targetdir;
    this->desiredVel = desiredVel;
    if (desiredVel > maxVel) {
      this->desiredVel = maxVel;
    }
  }
  
  void SApproachMovingBall::setParameters(const Vec& targetDriveDir,
                                          const Vec& targetHeading,
                                          double desiredVel) 
  throw(TribotsException)
  {
    approachFacing=targetHeading;
    approachDirection=targetDriveDir;
    this->desiredVel = desiredVel;
    if (desiredVel > maxVel) {
      this->desiredVel = maxVel;
    }
  }

  DriveVector SApproachMovingBall::getCmd(const Time& tt) 
    throw(TribotsException)
  {
    Time t;
    t=tt;
    
    t.add_msec(-30);        // \todo: Dirty hack, need to repair this



    double kp=20;
    double ki=0;
    double kd=10;
    double minturn=-60;
    double maxturn=60;

    headingController.set_new_parameters(kp,ki,kd,minturn,maxturn);



    Angle targetHeading=approachFacing.angle();
    
    Vec targetMovement=approachDirection;

   Vec ballvel = 
      WorldModel::get_main_world_model().get_ball_location(t).velocity.toVec();
    RobotLocation robotLocation = 
      WorldModel::get_main_world_model().get_robot_location(t);

    Angle robotHeading = robotLocation.heading;
    Vec ballPos = WorldModel::get_main_world_model().get_ball_location(t).pos.toVec();

    Vec robotPos=robotLocation.pos;
    
    Angle ballHeading=(ballPos-robotPos).angle();
    
    
    Vec robotVel=robotLocation.vtrans;
    Frame2d world2robot =  WhiteBoard::getTheWhiteBoard()->getAbs2RelFrame(t);
    Frame2d robot2world =  WhiteBoard::getTheWhiteBoard()->getRel2AbsFrame(t);

    Vec relBall=world2robot*ballPos;
    
//    cout<< "Sapproach Moving ball Relball.x "<< relBall.x <<" relball.y " << relBall.y<<endl;

	DriveVector dv;
    dv.kick = 0;
#if 0
    dv.vrot =                   // calculate correction of heading
      headingController.getAction(0,
                                  (robotHeading -
                                  (targetHeading-Angle::quarter)).get_rad_pi(),
                                  MWM.get_game_state().intended_cycle_time
                                  /*,LOUT*/);
  
    //dv.vrot = lowVelMod.getValue(fabs(dv.vrot)) * (fabs(dv.vrot) / dv.vrot);
#else    
    // Calculate an approaching vector
    double hdiff=(robotHeading - (targetHeading-Angle::quarter)).get_rad_pi()/M_PI;
    double max_rotate_speed=3.5;                              
    double balldiff=(robotHeading-(ballHeading-Angle::quarter)).get_rad_pi()/M_PI;
        
    if ((ballPos-robotPos).length()<700&&fabs(balldiff)<0.3){
    hdiff=balldiff;
    LOUT << "RiCHTE AUF BALL AUS"<< hdiff; 
    } 
    
    
    
    
    
    double PLF_V1x=10;
    double PLF_V1y=20;
    double PLF_V2x=20;
    double PLF_V2y=50;
    double PLF_V3x=100;
    double PLF_V3y=100;
    max_rotate_speed=3.5;


    PiecewiseLinearFunction plf;
    plf.addVertex(PLF_V1x,PLF_V1y);      // schneller Anstieg
    plf.addVertex(PLF_V2x,PLF_V2y);      // schneller Anstieg
    plf.addVertex(PLF_V3x,PLF_V3y);      // schneller Anstieg



//double differenzanteil=headingController.getAction(0,hdiff*M_PI,MWM.get_game_state().intended_cycle_time, 
//                                  LOUT);
//        LOUT << "\n DIfferenzanteil = " << differenzanteil; 
  

        dv.vrot = -(fabs(hdiff)/hdiff)*max_rotate_speed*plf.getValue(fabs(hdiff)); // +differenzanteil/2;

//cout <<"dv.vrot"<<dv.vrot<<endl;


//dv.vrot = -(fabs(hdiff)/hdiff)*max_rotate_speed*plf.getValue(fabs(hdiff));
    
        LOUT << "\nMax_speed="<<max_rotate_speed<< "plf="<<plf.getValue(fabs(hdiff))<<" VROT = " << dv.vrot; 
  
#endif
    Angle theta=(ballPos-robotPos).angle(targetMovement);
    
    if (ballvel.length()<1.0&&(ballPos-robotPos).length()<1000) {  // Ball bewegt sich nicht und ich bin nah dran
    desiredVel=1.8;
    }

    
    if (ballvel.length()>maxVel) {  // Ball bewegt sich zu schnell
      ballvel=maxVel*ballvel.normalize();
    }

    Vec evade = 1000.*approachTarget.getValue(fabs(theta.get_rad_pi())/M_PI)*
        targetMovement.normalize().rotate(M_PI-theta.get_rad_pi()*0.5);
        // \todo: Was macht das genau?
    Vec dogCurveTarget = ballPos +  evade;
    Vec relDCT = world2robot * dogCurveTarget; 

    
    LOUT << "\n% blue circle " << dogCurveTarget <<" 300 "
        << "\n% blue word "<<Vec(0,500)+dogCurveTarget 
                    <<" DCTGT\n" ;

    double dist=(ballPos-robotPos).length();
    double brake=approachSpeeds.getValue(dist/1000.);

    if (dist > 1000.) {
      brake = 1.;       // do not brake
    }

    double ballvelfactor=2.0;
    TUNABLE("BALLVELfactor",&ballvelfactor);
    
    Vec desiredVtrans = (world2robot*dogCurveTarget).normalize()*desiredVel*brake;

    if (ballvel.length() < 0.5) {
      
    } else {
      desiredVtrans += ballvelfactor * ballvel.rotate(-robotHeading);//add ball velocity (magic value)
    }

    // Wenn der ball sehr nahe und direkt zwischen den hoernchen liegt, dann 
    // fahre direkt los
    if (active && fabs(relBall.x) < 80. && relBall.y < 400. && relBall.y > 0. &&
        fabs(ballPos.x) < MWM.get_field_geometry().field_width/2. - 1000.) {
      LOUT << "Fast CATCH" << endl;
      if (ballvel.length() < 0.5) {
        desiredVtrans = Vec(0., 0.8*maxVel);
        LOUT << "SApproachMovingBall: ballvel to small, assuming 0" << endl;
      } else {
        desiredVtrans = Vec(0., 0.8*maxVel) + ballvelfactor * ballvel.rotate(-robotHeading);
        LOUT << "SApproachMovingBall: ballvel ok" << endl;
      }
    }
    
    if (desiredVtrans.length() > maxVel) { // cut vtrans at max speed
      desiredVtrans = desiredVtrans.normalize()*maxVel;
    }

    if (robotLocation.vtrans.length() < 0.6 && desiredVtrans.length() > 1.8) {
      desiredVtrans = desiredVtrans.normalize()*1.8;
    }
    
    // We now have our desired translational and rotational drive vectors, 
    // but now we may have to modify it in order evade obstacles...

    dv.vtrans = 
      WhiteBoard::getTheWhiteBoard()->calculateEvasiveMovement(desiredVtrans,
                                                              dogCurveTarget, 
                                                              tt);

    if (ausweichennaheball){
    double distToObstacle;
    distToObstacle= obstacle_distance_to_line (robotPos,ballPos+(robotPos-ballPos).normalize()*250, MWM.get_obstacle_location(t,false ));
    double maxdistToObstacleThresh=10;
    double distBallAusweichen=1000;
    
    TUNABLE ("MAXDOBSTHR",&maxdistToObstacleThresh);
    TUNABLE ("DISTBALLAUSW",&distBallAusweichen);
    
    
    if (relBall.length()<distBallAusweichen&&distToObstacle<maxdistToObstacleThresh)  
    {LOUT <<" HINDERNISS IM ANFAHRTBEREICH AUSWEICHEN"<<endl;
      
      dv.vtrans = ((relBall).normalize()*2.).rotate((ballPos-robotPos).angle(dogCurveTarget-robotPos).get_deg_180() > 0 ? Angle::quarter : -Angle::quarter);

    }
    }       

    
    if (relBall.y < 500 && relBall.y > 150. && relBall.length()<1000 &&
        ((dv.vtrans.x < -.3 && relBall.x < 100.) ||
        (dv.vtrans.x > +.3 && relBall.x > 100))) {
      LOUT<<"\n Ball Nicht richtig angefahren, bremse um besser drehen zu koenenn";
      dv.vtrans=dv.vtrans *0.6;
    }
    
    if (fabs(dv.vrot) > 1.8 && dv.vtrans.length() > 2.5) {
      dv.vtrans = dv.vtrans.normalize() * 2.5;
      LOUT << "SApproachMovingBall: Fahrtvektor verkuerzt, um hohe Drehgeschwindigkeit zu realisieren." << endl;
    }
    
    Vec absVtrans = dv.vtrans.rotate(robotHeading) * 1000;
    

    LOUT << "\%solid black " << LineSegment(robotPos, robotPos + absVtrans) << endl; 
  //  cout <<"dv.vrot"<<dv.vrot<<endl;
    return dv;
  }


void  SApproachMovingBall::updateTactics(const TacticsBoard& tb) throw()
{
string key="AusweichenNaheBall";
if (tb[key]== string("an")){
	ausweichennaheball=true;

}

if (tb[key]== string("aus")){
	ausweichennaheball=false;

}

if (tb["FastCatch"] == string("an")) {
  active = true;
}
else {
  active = false;
}


}
