#include "SDribbleBallToPos.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"
#include "../../../Fundamental/RemoteTune.h"

#include <iostream>
#include <cmath>

#define DEBUG_DRIBBLE 1      // sollte immer an bleiben ?!

#define HARD_TURNS_OFF 1

namespace Tribots
{
using namespace std;
  
SDribbleBallToPos::SDribbleBallToPos(double pullBackPositionXPosMax,
                                     double pullBackPositionYPosMax)
  : SDribbleBallToPosInterface("SDribbleBallToPos"), 
    pullBackPositionXPosMax(pullBackPositionXPosMax),
    pullBackPositionYPosMax(pullBackPositionYPosMax)
{}    
  
void 
SDribbleBallToPos::setParameters(const Vec& target, double transVel,
                                 bool forceHardTurn) throw(TribotsException)
{
  this->target = target;
  this->transVel = transVel;
  this->forceHardTurn = forceHardTurn;
}

DriveVector
SDribbleBallToPos::getCmd(const Time& tt) throw(TribotsException)
  {
    if (HARD_TURNS_OFF) forceHardTurn=0;
    // --------------------------------------------------------------------------------------------------------    
    // get WM-data  
    Frame2d world2robot =  WBOARD->getAbs2RelFrame(tt);
    Frame2d robot2world =  WBOARD->getRel2AbsFrame(tt);    
    
    // --------------------------------------------------------------------------------------------------------    
    // init local
    Vec _pullBackPosition;
    double _dDirectionDiff = ((world2robot*target).angle()-Angle::quarter).get_deg_180();
    
    if (DEBUG_DRIBBLE && forceHardTurn) LOUT << "hard turn forced!\n";
    
    // --------------------------------------------------------------------------------------------------------
    // calc PullBackPos      
    if ( (fabs(_dDirectionDiff) >= 45.0) || (forceHardTurn) ) {
      if (_dDirectionDiff>=0.0) _pullBackPosition = robot2world*(Vec(pullBackPositionYPosMax,pullBackPositionXPosMax));
      else _pullBackPosition = robot2world*(Vec(-pullBackPositionYPosMax,pullBackPositionXPosMax));
    }
    else {
      if (_dDirectionDiff>=0.0) _pullBackPosition =robot2world*(Vec((fabs(_dDirectionDiff)*pullBackPositionYPosMax)/45.0,pullBackPositionXPosMax));
      else _pullBackPosition = robot2world*(Vec(-(fabs(_dDirectionDiff)*pullBackPositionYPosMax)/45.0,pullBackPositionXPosMax));
    }
    
    // --------------------------------------------------------------------------------------------------------
    // calc RotVel
    double rotVel;
#ifdef USE_ODESIM
    double _f1 = 5.0;
#else
    double _f1 = 3.0; // work: default: 1.8
TUNABLE("ROT_VEL",&_f1);
LOUT << "###Tuning actual rotationspeed/_f1:" << _f1 ; 
#endif
//    double _f2 = 2.5;
    

double lfdbval1 = 0.1;
double lfdbval2 = 0.2;
double lfdbval3 = 0.5;
double lfdbval4 = 1.0;


TUNABLE("ROTngl1",&lfdbval1);
TUNABLE("ROTngl2",&lfdbval2);
TUNABLE("ROTngl3",&lfdbval3);
TUNABLE("ROTngl4",&lfdbval4);


    PiecewiseLinearFunction plf;
//    plf.addVertex(0.0,0.4);
//    plf.addVertex(0.1,0.4);
//    plf.addVertex(0.2,0.5);
//    plf.addVertex(0.3,0.5);
//    plf.addVertex(0.5,0.4);
//    plf.addVertex(0.7,0.4);
//    plf.addVertex(1,0.4);
        
    plf.addVertex(0.1,lfdbval1);
    plf.addVertex(0.2,lfdbval2);
    plf.addVertex(0.5,lfdbval3);
    plf.addVertex(1.,lfdbval4); 
  
    const RobotLocation& robot = MWM.get_robot_location(tt);
    double presentVelocity = robot.vtrans.length();  // now its in meters.
	    
    double rotfloat = _dDirectionDiff / 180.0;

    rotVel = plf.getValue(fabs(rotfloat))*_f1*rotfloat/fabs(rotfloat);
//    rotVel = plf.getValue(fabs(rotfloat))*_f1*rotfloat/fabs(rotfloat);
    
    /*
    if (_dDirectionDiff > 0.0) {
      rotVel = (fabs(_dDirectionDiff)*_f1)/180.0;
      if (fabs(rotVel)<1.0) rotVel=plf.getValue(rotVel);
      if (forceHardTurn) rotVel=180*_f2/180.0;
    }
    else {
      rotVel = -(fabs(_dDirectionDiff)*_f1)/180.0;
      if (fabs(rotVel)<1.0) rotVel=plf.getValue(rotVel);
      if (forceHardTurn) rotVel=-(180*_f2/180.0);      
    } */   
    
    if (DEBUG_DRIBBLE) {    
      LOUT<<"\n\% yellow thin solid circle "<<target.x<<" "<<target.y<<" 50 word "<<target.x+100<<" "<<target.y+100<<" Target\n";
      LOUT<<"\n\% light_blue thin solid circle "<<target.x<<" "<<target.y<<" 100 word "<<target.x+300<<" "<<target.y+100<<" WantedOrientation\n";
      LOUT<<"\n\% dark_green thin solid cross "<<_pullBackPosition.x<<" "<<_pullBackPosition.y<<" word "<<_pullBackPosition.x+100<<" "<<_pullBackPosition.y+100<<" PullBack("<<rotVel<<")\n";
    }

    DriveVector dv;
    
    dv.kick = 0;        
    dv.vrot=rotVel;    
    dv.vtrans = (world2robot*_pullBackPosition).normalize() * transVel;
    
    string relBall = MWM.get_message_board().scan_for_prefix("rel_ball");
    if (relBall != "") { // an robbi rucken, falls relative infos verfuegbar
      if (relBall == "rel_ball_left") {
        dv.vtrans = dv.vtrans * .5 + Vec(-1.5, 0.);
      }
      else if (relBall == "rel_ball_right") {
        dv.vtrans = dv.vtrans * .5 + Vec(1.5, 0.);
      }
    }    
    
    RobotLocation robotLocation = 
		    MWM.get_robot_location(tt);
    FieldGeometry fg = 
		    MWM.get_field_geometry();
    Vec pos=robotLocation.pos;
    Angle headangle=robotLocation.heading;
    Vec head(1,0);
    head=head.rotate( headangle+ Angle::quarter);
    
    Vec testPunkt;
    testPunkt=pos+head.normalize()*1500;
    LOUT << "\n% blue circle " << testPunkt<<" 60 "
		    << "\n% blue word "<<Vec(0,500)+testPunkt
		    <<" TestPunkt\n" ;

#if 1 
    if (fabs(testPunkt.x) >  fg.field_width/2 ){
	    if ((pos.x>0 && headangle.get_deg_180() > -90) ||
                (pos.x<0 && headangle.get_deg_180() > 90)){dv.vrot=1.5;
		dv.vtrans.x=0.5;dv.vtrans.y=2;}
	    else {dv.vrot=-1.5;
	    dv.vtrans.x=-0.5;dv.vtrans.y=2;}

        LOUT<<"RETTEN VOR DEM AUS!!!!!\n";
    
    }
#endif
    
    
    return dv;
  }
}

