
#include "BNewDribbleTest.h"

using namespace Tribots;
using namespace std;


/**
 * debug methode um die waypoints auszugeben
 **/
void BNewDribbleTest::print_way_points() {
	for(unsigned int i = 0; i<wayPoints.size(); i++){
		LOUT << "% red solid cross " << offset +wayPoints[i].pos << "\r\n";
	}
}

XYRectangle* BNewDribbleTest::get_dribble_area() {
	return new XYRectangle( offset, offset +Vec(fgeom.field_width/2, fgeom.field_length/2));
}

Vec* BNewDribbleTest::get_dribble_area_center() {
	return new Vec(offset + Vec(fgeom.field_width/4,fgeom.field_length/4));
}

/**
 * ueber diese methode wird eingestellt ob man in der linken
 * oder der rechten seite der eigenen haelfte dribbeln will
 * uebergeben wird: "dribbleLeftArea" oder "dribbleRightArea"
 **/
bool BNewDribbleTest::set_field_area(std::string area){
	LOUT << "setFieldArea wurde aufgerufen mit: " << area << " \r\n";
	if(area == "dribbleLeftArea") {
		leftArea = true;
		offset = Vec(-fgeom.field_width/2, -fgeom.field_length/2);
		return true;
	}
	if(area == "dribbleRightArea") {
		offset = Vec(0., -fgeom.field_length/2);
		leftArea = false;
		return true;
	}
	return false;
}


/**
 * dem konstruktor wird die dribblegeschwindigkeit in m/s uebergeben
 **/
BNewDribbleTest::BNewDribbleTest(double _dribblevel) : Behavior("BNewDribbleTest") {
  skill = new SDribbleBallToPos();
	fgeom = MWM.get_field_geometry();
  timer = 0;
  currentWayPoint = 0;
	leftArea = true;
	const double dist = 1500;								//abstand zu rand der eingehalten wird
	set_field_area("dribbleLeftArea");				//standardmaessig linke seite
	
  addWayPoint(Vec(fgeom.field_width/4, dist));
  addWayPoint(Vec(dist							 , fgeom.field_length/4));
  addWayPoint(Vec(fgeom.field_width/2 - dist, fgeom.field_length/4));
  addWayPoint(Vec(fgeom.field_width/4, fgeom.field_length/2 - dist));
  addWayPoint(Vec(dist							 ,  fgeom.field_length/4));
  addWayPoint(Vec(fgeom.field_width/4, fgeom.field_length/4));
  addWayPoint(Vec(fgeom.field_width/4, dist));
   
  dribblevel = _dribblevel;
  }

BNewDribbleTest::~BNewDribbleTest() throw () {
  delete skill;
}

bool BNewDribbleTest::checkCommitmentCondition(const Time& t) throw() {
  timer++;
  return checkInvocationCondition(t);    
}


/**
 * wenn ich in meiner Haelfte bin und wenn ich den Ball besitze
 * und wenn ich mich in TestState2 befinde
 **/
bool BNewDribbleTest::checkInvocationCondition(const Time& t) throw() {
	bool inCorrectArea   = false;
	bool inTestState = false;
	bool haveBall		 = false;
	XYRectangle correctArea = XYRectangle( offset, offset +Vec(fgeom.field_width/2, fgeom.field_length/2));
	LOUT << "% blue solid cross " << offset <<" blue solid cross "							//benutzes gebiet ausgeben
			 << offset +Vec(fgeom.field_width/2, fgeom.field_length/2) << "\r\n";
  inCorrectArea	= correctArea.is_inside(MWM.get_robot_location(t).pos);
	haveBall			= WBOARD->doPossessBall(t);
	inTestState		= MWM.get_game_state().refstate == testState2;
	
	if(inTestState){
		LOUT << "BNewDribbleTest: \r\n";
		if(!inCorrectArea) LOUT << "I'm not in the correct Area \r\n";
		if(!haveBall)	 LOUT << "Do not have a Ball \r\n";
	}
	
	return inTestState && inCorrectArea && haveBall;
}

/**
 * fahre alle waypoints ab
 **/
DriveVector BNewDribbleTest::getCmd(const Time& t) throw(TribotsException) {
  
  robot				= MWM.get_robot_location(t);
  
  if (wayPoints[currentWayPoint].area.is_inside(robot.pos -offset)) {
    if (currentWayPoint==(int)wayPoints.size()-1) currentWayPoint = 0;
    else currentWayPoint++;
  }
	
	print_way_points();
  
  LOUT<<"driving to WayPoint #"<<currentWayPoint<<endl;
  wayPoints[currentWayPoint].area.draw(LOUT);
  
  skill->setParameters(offset + wayPoints[currentWayPoint].pos, dribblevel , 0);
	
  return skill->getCmd(t);
}


/* wird gerade nicht benutzt, kann nur mit trainer wieder verwendet werden
void BNewDribbleTest::updateTactics (const TacticsBoard& tb) throw ()
{
  string tmpstr="";
  tmpstr = tb[string("DribbleTestVel")];
  if (tmpstr != "") {
    dribblevel = atof(tmpstr.c_str()); 
  }
}
*/

void BNewDribbleTest::gainControl(const Time& t) throw(TribotsException) {
  timer=0;
}

void BNewDribbleTest::loseControl(const Time& t) throw(TribotsException) {
  timer=0;
}
  

void BNewDribbleTest::addWayPoint(Vec pos) {
  struct wayPoint _wP;
  _wP.pos=pos;
  _wP.area=XYRectangle(Vec(pos.x-600,pos.y-600),Vec(pos.x+600,pos.y+600));
  wayPoints.push_back(_wP);
}
