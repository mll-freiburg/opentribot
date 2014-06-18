#include "SPass.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>

using namespace Tribots;
using namespace std;

SPass::SPass(int kickDuration): Skill("SPass"), kickDuration(kickDuration)
{
	hdiff=0;
	active=false;
	skill = new SDribbleBallToPos();
	target	= Vec(0,0);
	emergencyKick = false;
}

SPass::~SPass() throw()
{
	if (skill) {
		delete skill;
	}
}

void SPass::cycleCallBack(const Time& t) throw() {
/*	if (active) {
		calcHeadingDifference(t);
	}*/
}

void SPass::calcHeadingDifference(const Time& t) throw() {
	const RobotLocation& robot = MWM.get_robot_location(t);
	Angle robotHeading = robot.heading;
	Angle targetHeading = (target - robot.pos).angle();

	//assimodulo
	double newhdiff = (robotHeading - (targetHeading-Angle::quarter)).get_deg();
	if (newhdiff > 180) {
		newhdiff -= 360;
	}

	if (hdiff*newhdiff < 0 && fabs(hdiff-newhdiff) < 180 ) { // vorzeichen gewechselt und nicht überschlagen von 180 <-> -180
		LOUT << "SPass: Achtung: über das Ziel hinaus gedreht. Notschuss." << endl;
		emergencyKick = true;
	}
	hdiff = newhdiff;
}

void SPass::gainControl(const Time& t) throw(TribotsException) {
	active = true;
	hdiff=0;
}

void SPass::loseControl(const Time& t) throw(TribotsException) {
	active = false;
}


DriveVector SPass::getCmd(const Time& t) throw(TribotsException){
	//TODO: handle situations too close to the target


        LOUT << "% white solid circle  " << Circle(target, 600.) << endl; // Pass moeglich

	//calculate reinforcement learned action
  DriveVector dv = skill->getCmd(t);

//	if (hdiff ==0) {
//		LOUT << "SPass: hdiff is 0, recalculating" <<endl;
	calcHeadingDifference(t);
//	}

	LOUT << "SPass: Angle to destination in deg: " << hdiff << ", left threshold: " << shootErrorLeft << ", right threshold: " << shootErrorRight << "\n";
 
	if (((hdiff > -shootErrorLeft) && (hdiff < shootErrorRight)) || emergencyKick) {
		LOUT << "SPass: kicking with duration: " << kickDuration << "\n";
		LOUT << "SPass: EmergencyKick: " << (emergencyKick ? "TRUE" : "FALSE") << endl;
		emergencyKick = false;

		//dv.vtrans = robot.vtrans / robot.heading;
		//dv.vrot = 0;
		dv.kick = 2; //assume pass

		//TODO: adjust kick
		dv.klength = kickDuration;

		//notify ball loss
		WBOARD->resetPossessBall();
	}

  return dv;
}

void 
SPass::setParameters(const Vec& target, double transVel, double shootErrorLeft, double shootErrorRight) throw(TribotsException)
{
	bool b=0;
	skill->setParameters(target, transVel, b);
	this->target = target;
	
	this->shootErrorLeft = shootErrorLeft;
	this->shootErrorRight = shootErrorRight;
}
