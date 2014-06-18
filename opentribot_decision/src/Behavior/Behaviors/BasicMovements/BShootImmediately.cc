#include "BShootImmediately.h"

#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"

using namespace std;
namespace Tribots
{

long BShootImmediately::lastActivation = 0;

BShootImmediately::BShootImmediately(const string name, int hackKickLength) 
  : Behavior(name), hackKickLength(hackKickLength)
{}

BShootImmediately::~BShootImmediately() throw()
{}

bool 
BShootImmediately::checkCommitmentCondition(const Time& t) throw()
{
  return BShootImmediately::checkInvocationCondition(t);
}

bool 
BShootImmediately::checkInvocationCondition(const Time& t) throw()
{
  if (MWM.get_game_state().cycle_num - lastActivation < 4) {      
    // war gerade erst ausgelöst
    return false;
  }
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  return true;
}

DriveVector 
BShootImmediately::getCmd(const Time& t) throw(TribotsException)
{
  const RobotLocation& lrobot = MWM.get_robot_location(t);
  const FieldGeometry& fgeom  = MWM.get_field_geometry();
  double shoot_height = 750;
  bool reachable = false;	
  DriveVector dv;
  dv.vrot = 0;
  dv.kick = 1;
  dv.vtrans = lrobot.vtrans/lrobot.heading;
  
  
  LOUT << "BShootImmediately::getCmd: kick\n";
    Line goal_line = Line( Vec(-fgeom.field_width/2, fgeom.field_length/2), 
			   Vec( fgeom.field_width/2, fgeom.field_length/2));
    Line robot_orientation = Line( lrobot.pos, WBOARD->getRel2AbsFrame(t) * Vec(0,1000));
    Vec goal_line_point;
    
    try {
      goal_line_point = intersect(goal_line, robot_orientation);
    }catch(invalid_argument& e){
      LOUT << e.what() << "\n";
    }
  
  // wenn ich den neuen Kicker habe

    double dist;
    dist=(lrobot.pos-goal_line_point).length();
 	double grenze=7200;
	dv.klength = 45;
	

	if (dist < grenze ) {
		 dv.klength=35;
		
	}
	grenze=5000;
	if(dist < grenze){
	dv.klength = 30;
	}
	cout << "KICK "<<dist<< " *************** weiter als "<<grenze<<" meter"<<endl;
	

  
  LOUT << "\% red thick circle " << lrobot.pos << " 1000." << endl;
  LOUT << "\% red thick circle " << lrobot.pos << " 900." << endl;
  LOUT << "\% red thick circle " << lrobot.pos << " 800." << endl;
  
  lastActivation = MWM.get_game_state().cycle_num;   
  
  LOUT << "Gewaehlte Schussstaerke: " << dv.klength << endl;
  WBOARD->resetPossessBall();
  return dv;
}

};
