
#include "KickerDriveTest.h"

#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "WhiteBoard.h"
#include "../Fundamental/RemoteTune.h"
#include <cmath>

using namespace Tribots;
using namespace std;



KickerDriveTest::KickerDriveTest (const ConfigReader& cfg) throw () {
  WBOARD->readConfigs (cfg);
	lastKick.update();
	kicked = false;

}

DriveVector KickerDriveTest::process_drive_vector (Time t) throw () {
	DriveVector dv;
	unsigned char kick_time = 0;
	bool reachable;
	int dista =3;
	double angle_diff = 0.;
	int frot = 0;
	Vec goal_loc = Vec(0, MWM.get_field_geometry().field_length / 2.0);
	Vec rel_goal_loc = WBOARD->getAbs2RelFrame(t) * goal_loc;
	
        
        //AUSRICHTUNG ZUM TOR
	angle_diff = (rel_goal_loc.angle() - MWM.get_robot_location(t).heading - Angle::quarter).get_rad_pi() / M_PI;
        
        LOUT <<angle_diff<< "             !! " << endl ;
        
       
        if(angle_diff > 0.01) {
		frot =1;
                LOUT << "  01_05 => -1 " << endl;
      	}
	if(angle_diff < -0.01){
               frot =-1;
               LOUT << "  05_09 => 1 " << endl;
	}

        
        
        
        
        kick_time = 	WBOARD->getKickLength(rel_goal_loc.length(), 0.0, &reachable);
		
	
	// Spielzustand laut Refereebox
	int gamestate = WorldModel::get_main_world_model().get_game_state().refstate; 
	
	if(!(gamestate == stopRobot)){
	switch(gamestate){
	case 24 : dista=1 ; break;
	case 25 : dista=2 ; break;
	case 26 : dista=3 ; break;
	case 27 : dista=4 ; break;
	case 28 : dista=5 ; break;
	case 29 : dista=6 ; break;
	case 30 : dista=7 ; break;
	}
	}

  	LOUT << "gamestate: "<<gamestate << "   " ;
	// StopState
	if(gamestate == stopRobot) {		// nichts tun
		LOUT << "wir sind aus";
		dv.kick = 0;                   // nicht kicken
		dv.vtrans = Vec(0.,0.);        // keine translatorische Bewegung
		dv.vrot = 0;    // keine Drehbewegung
		return dv;               
		kicked=false;
	}

	if(!kicked){
		dv.vtrans = Vec(0,1); 
		dv.vrot = frot;
		dv.kick = 0;   
		if(rel_goal_loc.length()<dista*1000){
				kick_time = WBOARD->getKickLength(rel_goal_loc.length(), 0.0, &reachable);
				dv.kick =  HIGHKICK;
				kicked = true;
				LOUT << "kicked from: " << rel_goal_loc.length() << endl;
				LOUT << "was reachable: " << reachable << endl;
				LOUT << "kicklength was: " << (int) kick_time << endl;
				dv.klength = kick_time;
				return dv;
		               	}
		return dv;
             	}else{dv.vtrans = Vec(0,0); 
				dv.vrot = frot;
				dv.kick = 0;
				return dv;
				LOUT << "was kicked";}}

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("KickerDriveTest"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new KickerDriveTest (reader);
    }
  };
  Builder the_builder;
}

