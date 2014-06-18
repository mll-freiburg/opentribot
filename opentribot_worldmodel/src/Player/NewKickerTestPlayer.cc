
#include "NewKickerTestPlayer.h"

#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "WhiteBoard.h"
#include "../Fundamental/RemoteTune.h"
#include <cmath>
#include <stdlib.h>
using namespace Tribots;
using namespace std;


/**
 *	Dies ist ein einfacher Player um den neuen Kicker mit
 *	verschiedenen Parametern zu testen. 
 *	Die verschiedenen Parameter koennen im Quellcode (Constructor) gesetzt werden
 *oder ueber den trainer (siehe unten)
 *
 * Es gibt zwei Testzustaende:
 *
 *	testState1:
 *		man kann den roboter an eine beliebige stelle
 *		wenn er den ball fuer mehr als zwei sekunden besitzt
 *		versucht er den ball auf der vorher gesetzten hoehe ins tor zu schiessen
 *
 * testState2:
 *		wenn der roboter den ball laenger als zwei sekunden besitzt
 *		der roboter faehrt von der im constructor gesetzen position mit der 
 *		eingestellten geschwindigkeit auf die im constructor gesetzte schussposition
 *		und schiesst. kehrt dann zurueck auf warteposition
 *
 * geschwindigkeit kann ueber trainer eingestellt werden:
 *    "langsamer" -> velocity = 1.5;
 *		"langsam"   -> velocity = 2.0;
 *		"normal"		-> velocity = 2.3;
 *		"schneller" -> velocity = 2.8;
 *
 *	 kick_height = "penalty_shoot_height"
 *		jeweils wird damit der y wert der positionen gesetzt, x ist immer 0
 *	 shoot_pos = "GoalieAttackArea"
 *  start_pos = "obstacles_behind_goal_distance"
 *
 *
 */


NewKickerTestPlayer::NewKickerTestPlayer(const ConfigReader& cfg) throw () {
    WBOARD->readConfigs( cfg );
    debug = true; // messages anschalten


    //hier stellt er sich bei testState1 auf
    start_pos = Vec( 0., -3000. );
    // anfahrtsgeschwindigkeit
    velocity = 2.5;
    //ab hier schiesst er
    shoot_pos = Vec( 0., 2000. );
    // auf dieser hoehe geht der ball ins tor
    kick_height = 800.;

	 // TODO langsamer machen
    goto_pos_skill = new SPhysGotoPosAvoidObstacles( );
    goto_pos_skill->init( Vec( start_pos ), Angle::deg_angle( 0. ), true, true, true, false );
    dribble_skill = new SDribbleBallToPosRL( );
    dribble_skill->setParameters( shoot_pos, velocity, false );
}

void NewKickerTestPlayer::updateTactics(const TacticsBoard& tb)
throw () {
	 
	 kick_height = atof(tb[string("penalty_shoot_height")].c_str());
	 shoot_pos = Vec(0., atof(tb[string("GoalieAttackArea")].c_str()));
	 start_pos = Vec(0., atof(tb[string("obstacles_behind_goal_distance")].c_str()));

	 goto_pos_skill->init( Vec( start_pos ), Angle::deg_angle( 0. ), true, true, true, false );
    dribble_skill->setParameters( shoot_pos, velocity, false );
	 
    // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
    if (tb[string( "AllgemeineFahrgeschwindigkeit" )] == "langsamer") {
        velocity = 1.5;
    } else
        if (tb[string( "AllgemeineFahrgeschwindigkeit" )] == "langsam") {
        velocity = 2.0;
    } else
        if (tb[string( "AllgemeineFahrgeschwindigkeit" )] == "schneller") {
        velocity = 2.8;
    } else { // =normal
        velocity = 2.3;
    }
	 dribble_skill->setParameters( shoot_pos, velocity, false );

	 
	 if (debug) LOUT << "velocity changed to: " << velocity << " \r\n";
	 if (debug) LOUT << "kick_height changed to: " << kick_height << " \r\n";
	 if (debug) LOUT << "start_pos changed to: " << start_pos << " \r\n";
	 if (debug) LOUT << "shoot_pos changed to: " << shoot_pos << "\r\n";
}

DriveVector NewKickerTestPlayer::process_drive_vector(Time t) throw () {
    DriveVector dv;
    unsigned char kick_length = 0;
    bool reachable;
    int gamestate = WorldModel::get_main_world_model( ).get_game_state( ).refstate;
    Vec goal_loc = Vec( 0, MWM.get_field_geometry( ).field_length / 2.0 );
    const RobotLocation& rob_loc = MWM.get_robot_location( t );
    double goal_distance = (goal_loc - rob_loc.pos).length( );


    // StopState
    if (gamestate == stopRobot) { // nichts tun
        dv.kick = 0; // nicht kicken
        dv.vtrans = Vec( 0., 0. ); // keine translatorische Bewegung
        dv.vrot = 0; // keine Drehbewegung
    }

    kick_length = WBOARD->getKickLength( goal_distance, kick_height, &reachable );
    LOUT << "Berechne Kick fuer Abstand: " << goal_distance << " mm \n";
    LOUT << "Kicklaenge: " << (int) kick_length << "\n";


    // statischer kicktest
    if (gamestate == testState1) {
        if (WBOARD->doPossessBall( t )) {
            if (have_ball.elapsed_sec( ) > 2) {
                dv.kick = HIGHKICK;
                dv.klength = kick_length;
            }
        }else {
            if (debug) LOUT << "Do not have a Ball \r\n";
            have_ball.update( );
        }

    }



    // kicktest mit anfahrt
    if (gamestate == testState2) {

        if (!WBOARD->doPossessBall( t )) {
            dv = goto_pos_skill->getCmd( t );
            have_ball.update( );
        }else {
            if (have_ball.elapsed_sec( ) > 2) {
                if ((rob_loc.pos - shoot_pos).length( ) < 1000.) {
                    dv.kick = HIGHKICK;
                    dv.klength = kick_length;
                    LOUT << "kicked from: " << goal_distance << " mm distance" << endl;
                    LOUT << "velocity was: " << rob_loc.vtrans << endl;
                    LOUT << "ball sollte auf hoehe: " << kick_height << " ins tor gehen" << endl;
                    LOUT << "was reachable: " << reachable << endl;
                    LOUT << "kicklength was: " << (int) kick_length << endl;

                }else {
                    dv = dribble_skill->getCmd( t );
                }

            }
        }

    }

    return dv;
}


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {

    class Builder : public PlayerBuilder {
        static Builder the_builder;
    public:

        Builder() {
            PlayerFactory::get_player_factory( )->sign_up( string( "NewKickerTestPlayer" ), this );
        }

        PlayerType* get_player(const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException, bad_alloc) {
            return new NewKickerTestPlayer( reader );
        }
    };
    Builder the_builder;
}

