
#include "Coach.h"
#include "../Policies/PolicyFactory.h"
#include "../States/RemoteBlackboard.h"
#include "../../../../Fundamental/stringconvert.h"
#include <algorithm>
#include <cmath>

using namespace TribotsTools;
using namespace Tribots;
using namespace std;


Coach::Coach (const Tribots::ConfigReader& cfg1) throw (Tribots::TribotsException) : cfg (cfg1), policy(NULL), ignore_broadcast_prefix (0) {
  owner_bonus=0;//Owner Bonus ist 0 am anfang, deswegen gibt es keine Hysterese beim 1.wechsel.(Tmc200_0)
  last_ball_owner=0;
  cfg.get ("Coach::ball_position_mode", REMBB.coach_state.ball_position_mode);
  cfg.get ("Coach::ball_posession_mode", REMBB.coach_state.ball_posession_mode);
  cfg.get ("Coach::broadcast_mode",REMBB.coach_state.broadcast_mode);
  cfg.get ("Coach::teammate_mode",REMBB.coach_state.teammate_mode);
  cfg.get ("Coach::policy", REMBB.coach_state.policy_name);
  policy = PolicyFactory::get_policy_factory()->get_policy (std::string("---"), cfg);
  PolicyFactory::get_policy_factory()->policy_list (REMBB.coach_state.list_policies);

  Tribots::read_tactics (REMBB.coach_state.tactics_attributes, cfg);
  Tribots::make_default_tactics_board (REMBB.coach_state.tactics_board, REMBB.coach_state.tactics_attributes);
}


Coach::~Coach () throw () {
  if (policy)
    delete policy;
}


void Coach::update () throw () {
  if (REMBB.coach_state.sl_mirror_hint_mode) sl_mirror_hint ();
  if (REMBB.coach_state.ball_position_mode) ball_position ();
  if (REMBB.coach_state.ball_posession_mode) ball_posession ();
  if (REMBB.coach_state.broadcast_mode) broadcast ();
  // Teammate mode wird in den RemoteRobot.cpp bearbeitet!
  extra_message ();
  if (!policy || REMBB.coach_state.policy_name!=std::string(policy->get_name ())) {
    // Strategie wechseln
    try{
      Policy* new_policy = PolicyFactory::get_policy_factory()->get_policy (REMBB.coach_state.policy_name, cfg);
      if (policy)
        delete policy;
      policy = new_policy;
    }catch(Tribots::TribotsException& e){
      std::cerr << "Strategiewechsel fehlgeschlagen.\n" << e.what() << std::endl;
    }catch(std::exception& e) {
      std::cerr << "Strategiewechsel fehlgeschlagen.\n" << e.what() << std::endl;
    }
  }
  if (policy) {
    policy->observe ();
    policy->update ();
  }
}


void Coach::ball_position () {
  bool ball_found=false;
  unsigned int nearest_robot=99999;
  double smallest_dist=1e99;
  unsigned int num_robots = REMBB.robot_state.size();
  for (unsigned int sender=0; sender<num_robots; sender++)
    if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted)
      if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
        double len = (REMBB.robot_state[sender].ball_pos.pos-REMBB.robot_state[sender].robot_pos.pos).length();
        if (len<smallest_dist) {
          smallest_dist=len;
          nearest_robot=sender;
          ball_found=true;
        }
      }
  if (ball_found) {
    Vec comm_ball = REMBB.robot_state[nearest_robot].ball_pos.pos.toVec();
    for (unsigned int receiver=0; receiver<num_robots; receiver++) {
      if (receiver!=nearest_robot)
        REMBB.robot_state[receiver].message_board.publish_stream() << "Ball: " << static_cast<int>(comm_ball.x) << ' ' << static_cast<int>(comm_ball.y) << '\n';
    }
  }
}

void Coach::sl_mirror_hint () {
  // Ballpositionen herholen
  vector<Vec> ball_positions;
  vector<unsigned int> sender_index_ball_positions;
  unsigned int num_robots = REMBB.robot_state.size();
  for (unsigned int sender=0; sender<num_robots; sender++) {
    if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted) {
      if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
        ball_positions.push_back (REMBB.robot_state[sender].ball_pos.pos.toVec());
        sender_index_ball_positions.push_back (sender);
      }
    }
  }
  
  // Ballpositionen clustern (single link, nur bi 1m Abstand)
  const unsigned int num=ball_positions.size();
  vector<unsigned int> clusters (num);
  for (unsigned int i=0; i<num; i++) {
    clusters[i]=i;
  }
  for (unsigned int i=0; i<num; i++) {
    for (unsigned int j=i+1; j<num; j++) {
      if ((ball_positions[i]-ball_positions[j]).squared_length()<1e6 && clusters[i]!=clusters[j]) {
        for (unsigned int k=0; k<num; k++) {
          if (clusters[k]==clusters[j]) {
            clusters[k]=clusters[i];
          }
        }
      }
    }
  }
  
  // zaehlen, wie viele Roboter Ball in einem der Cluster sehen
  vector<unsigned int> clusternum (num);
  for (unsigned int i=0; i<num; i++) {
    clusternum[i]=0;
  }
  for (unsigned int i=0; i<num; i++) {
    clusternum[clusters[i]]++;
  }
  
  // Cluster mit groesster und Cluster mit zweitgroesster Anzahl Roboter suchen
  unsigned int num_max=0, num_snd_max=0;
  unsigned int index_max=0, index_snd_max=0;
  for (unsigned int i=0; i<num; i++) {
    if (clusternum[i]>num_max) {
      num_snd_max=num_max;
      index_snd_max=index_max;
      num_max=clusternum[i];
      index_max=i;
    } else if (clusternum[i]>num_snd_max) {
      num_snd_max=clusternum[i];
      index_snd_max=i;
    }
  }
  if (num_max>num_snd_max && num_snd_max>0) {
    // es gibt eine Mehrheitsentscheidung und mindestens ein Roboter, der den Ball falsch sieht
    Vec pos (0,0);
    for (unsigned int i=0; i<num; i++) {
      if (clusters[i]==index_max) {
        pos+=ball_positions[i];
      }
      pos/=static_cast<double>(num_max);
    }
    unsigned int i=0;
    for (unsigned int sender=0; sender<num_robots; sender++) {
      if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted) {
        if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
          if (clusters[i]!=index_max) {
            // Roboter sender moeglicherweise mit Spiegelposition
            REMBB.robot_state[sender].sl_mirror_hint_request=true;
            REMBB.robot_state[sender].sl_mirror_hint=pos;
          }
          i++;
        }
      }
    }
  }
}

void Coach::ball_posession () {
  { // ownsBall bis 500 mm Abstand
    bool team_owns = false;
    bool team_owns_extended = false;
    unsigned int owner = 99999;
    unsigned int num_robots = REMBB.robot_state.size();
    for (unsigned int sender=0; sender<num_robots; sender++) {
      if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted) {
        if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
          Vec robot_ball = (REMBB.robot_state[sender].ball_pos.pos.toVec()-REMBB.robot_state[sender].robot_pos.pos)/REMBB.robot_state[sender].robot_pos.heading;


          bool possessBallSent = REMBB.robot_state[sender].message_board.scan_for_prefix("pB!").length() > 0;
          if (possessBallSent && 
              robot_ball.length()<500 && (robot_ball.angle()-Angle::quarter).in_between(-Angle::sixth, Angle::sixth))  {
            team_owns = true;
            owner = sender;
          }
          if (REMBB.robot_state[sender].message_board.scan_for_prefix("extendedPB!").length() > 0) {
            team_owns_extended = true;
          }
        }
        
        for (unsigned int receiver=0; receiver<num_robots; receiver++) {
          if (team_owns && receiver!=owner) REMBB.robot_state[receiver].message_board.publish_stream() << "OwnsBall!\n";
          if (team_owns_extended && receiver!=owner) { REMBB.robot_state[receiver].message_board.publish_stream() << "OwnsBallExtended!\n"; /*cerr << "OwnsBallExtended" << endl;*/ }
        }
        if (team_owns) return ;  // do not have to check for near ball or ball possession in other senders, since only first owner wins ;-)
      }
    }
  }



  double verteidigungslinie = 0.;
  
  { 
    // nearBall bis 2000 mm Abstand
//    bool team_owns = false;
    unsigned int owner = 99999;
    unsigned int num_robots = REMBB.robot_state.size();
    int closest_robot=-1;
    vector<int> possibleRobotsAttack;
    vector<int> possibleRobotsDefendPri1;
    vector<int> possibleRobotsDefendPri2;
    vector<double> distances(num_robots);
        
    double closest_distance=999999;
    for (unsigned int sender=0; sender<num_robots; sender++)
    {
      if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted){
        if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
	  Vec robot_pos = REMBB.robot_state[sender].robot_pos.pos * REMBB.robot_state[sender].own_half;
	  Vec ball_abs = REMBB.robot_state[sender].ball_pos.pos.toVec() * REMBB.robot_state[sender].own_half;

	  Angle robot_heading = REMBB.robot_state[sender].robot_pos.heading + (REMBB.robot_state[sender].own_half < 0 ? Angle::half : Angle::zero);
	  Vec robot_ball_abs = ball_abs-robot_pos;
          Vec robot_ball_rel = robot_ball_abs/robot_heading;
	  Vec ball_own_goal = Vec(0,-REMBB.team_state.field_geometry.field_length/2.) -ball_abs;
	  Vec ball_opp_goal = ball_abs-Vec(0,REMBB.team_state.field_geometry.field_length/2. );
	  
	  /*
          cerr << robot_pos << " Heading: " << robot_heading.get_deg_180() << " rb_abs: "
               << robot_ball_abs << " rb_rel: " << robot_ball_rel << " bog: " 
               << ball_opp_goal << endl; */

	  distances.at(sender) = robot_ball_rel.length();

	  if (robot_ball_rel.length()<closest_distance){
	    closest_distance=robot_ball_rel.length();
	    closest_robot=sender;
	  }
	  // Angriffsfall
	  if (robot_ball_rel.length()<2000 && (robot_ball_rel.angle()-Angle::quarter).in_between(-Angle::eighth, Angle::eighth)) {
	    possibleRobotsAttack.push_back(sender);
	  }
	  /*	  cerr << "Winkel zu bowg: " << ball_own_goal.angle(-robot_ball_abs).get_deg_180() << " Winkel zu bopg: " << ball_opp_goal.angle(-robot_ball_abs).get_deg_180() << endl; */


	  // Verteidigungfall
	  if (robot_ball_rel.length()<2000 && ( robot_ball_rel.angle()-Angle::quarter).in_between(-Angle::sixth, Angle::sixth) 
	      && fabs(ball_own_goal.angle(-robot_ball_abs).get_deg_180())<30 ) {
	    possibleRobotsDefendPri1.push_back(sender);	  
	  }
	  else if (robot_ball_rel.length()<2000 && ( robot_ball_rel.angle()-Angle::quarter).in_between(-Angle::eighth, Angle::eighth) 
	           && ball_own_goal.angle(-robot_ball_abs).get_deg_180() * (robot_pos.x<0?-1:1) > 0
		   && ball_opp_goal.rotate(Angle::deg_angle(15 * (robot_pos.x<0?-1:1))).angle(-robot_ball_abs).get_deg_180() *(robot_pos.x<0?-1:1) < 0) {
            possibleRobotsDefendPri1.push_back(sender);
	  }
	  else if (robot_ball_rel.length()<1000 && ( robot_ball_rel.angle()-Angle::quarter).in_between(-Angle::eighth, Angle::eighth) && !
	           (fabs(ball_own_goal.angle(robot_ball_abs).get_deg_180())<30)) {
            possibleRobotsDefendPri2.push_back(sender);
	  }
	}
      }
    }
    
    if (closest_robot < 0) {
      return;
    }
    int own_half = REMBB.robot_state[closest_robot].own_half;
    bool verteidigungsfall = REMBB.robot_state[closest_robot].ball_pos.pos.y * 
      own_half < verteidigungslinie;
    
    if (verteidigungsfall) {
      if (possibleRobotsDefendPri1.size() > 0) {
	owner = possibleRobotsDefendPri1[0];
	closest_distance = distances[possibleRobotsDefendPri1[0]];
	for (unsigned int i=1; i < possibleRobotsDefendPri1.size(); i++) {
	  if (closest_distance > distances[possibleRobotsDefendPri1[i]]) {
	    owner = possibleRobotsDefendPri1[i];
	    closest_distance = distances[possibleRobotsDefendPri1[i]];
	  }
	}
      }
      else if (possibleRobotsDefendPri2.size() > 0) {
	owner = possibleRobotsDefendPri2[0];
	closest_distance = distances[possibleRobotsDefendPri2[0]];
	for (unsigned int i=1; i < possibleRobotsDefendPri2.size(); i++) {
	  if (closest_distance > distances[possibleRobotsDefendPri2[i]]) {
	    owner = possibleRobotsDefendPri2[i];
	  closest_distance = distances[possibleRobotsDefendPri2[i]];
	  }
	}
      }
    }
    else {
      if (possibleRobotsAttack.size() > 0) {
	owner = possibleRobotsAttack[0];
	closest_distance = distances[possibleRobotsAttack[0]];
	for (unsigned int i=1; i < possibleRobotsAttack.size(); i++) {
	  if (closest_distance > distances[possibleRobotsAttack[i]]) {
	    owner = possibleRobotsAttack[i];
	    closest_distance = distances[possibleRobotsAttack[i]];
	  }
	}
      }
    }
    if ((verteidigungsfall && possibleRobotsDefendPri1.size() + possibleRobotsDefendPri2.size() > 0) || (!verteidigungsfall && possibleRobotsAttack.size() > 0)) {
      for (unsigned int receiver=0; receiver<num_robots; receiver++)
	if (receiver!=owner) REMBB.robot_state[receiver].message_board.publish_stream() << "NearBall!\n";
    }
  }
#if 0 
  { 
  bool team_owns=false;
  unsigned int owner=99999;
  double highest_rank=0; //  ranking, siehe Methode go_to_ball_ranking(int player)
  unsigned int num_robots = REMBB.robot_state.size();
  for (unsigned int sender=0; sender<num_robots; sender++)
    if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted)
      if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
	  double rank=go_to_ball_ranking(sender);
	  if (rank>highest_rank) {
          highest_rank=rank;
	  team_owns=true;
	  owner=sender;
	  
	  }
      }
      if (last_ball_owner!=owner){
	      owner_bonus=500;
      }
      //cout <<"Owner = "<<owner<<endl;;	  
     //cout <<"HighestRank: "<< highest_rank <<endl;     
      if (team_owns) {
       for (unsigned int receiver=0; receiver<num_robots; receiver++)
       {			
       	
       if (highest_rank>5000&&receiver!=owner) REMBB.robot_state[receiver].message_board.publish_stream() << "NearBall!\n";
       }	
       
	}
     
  }
#endif

}


void Coach::broadcast () {
  unsigned int num_robots = REMBB.robot_state.size();
  for (unsigned int sender=0; sender<num_robots; sender++) {
    if (REMBB.robot_state[sender].in_game && !REMBB.robot_state[sender].comm_interrupted) {
      const vector<string>& msg (REMBB.robot_state[sender].message_board.get_incoming());
      for (unsigned int i=0; i<msg.size(); i++) {  // bearbeitet die i-te Nachricht des sender
        bool bc = true;
        for (unsigned int k=0; k<ignore_broadcast_prefix.size(); k++) {
          if (prefix (ignore_broadcast_prefix[k], msg[i])) {
            bc=false;
            break;
          }
        }
        for (unsigned int receiver=0; receiver<num_robots; receiver++) {
          if (sender!=receiver) {
            REMBB.robot_state[receiver].message_board.publish (msg[i]);
          }
        }
      }
    }
  }
}


void Coach::extra_message () {
  std::string& msg (REMBB.coach_state.extra_message);
  if (msg.size()>0) {
    for (unsigned int receiver=0; receiver<REMBB.robot_state.size(); receiver++)
      REMBB.robot_state[receiver].message_board.publish (msg);
    msg="";
  }
}

/**Diese Methode dient dazu ein Ranking für die Spieler zu berechnen, wer am besten an den Ball geht.In dieses Ranking fliessen ein: Entfernung zum Ball, relative Position Spieler/Ball/Tor, Ballgeschwindigkeit, Ausrichtung des Spielers, Hindernisse usw*/
double Coach::go_to_ball_ranking(int id){
	double ranking=0;
	
	//**************************************************************************/
	// simples Entfernungsranking (der mit der geringsten Entfernung bekommt das höchste Ranking)
	double dist_ball = ((REMBB.robot_state[id].ball_pos.pos)-REMBB.robot_state[id].robot_pos.pos).length(); ranking+=5000-dist_ball; 
	//**************************************************************************/
        if(REMBB.robot_state[id].ball_pos.pos.y<REMBB.robot_state[id].robot_pos.pos.y)ranking=ranking-2000;



	
	//**************************************************************************/
// 	//die Ausrichtung zum Tor ist auch Wichtig! und kann bis zu 1.5 m Entfernungsunterschied ausgleichen!
	Vec AbsBallfromRobot=REMBB.robot_state[id].own_half*(REMBB.robot_state[id].ball_pos.pos.toVec()-REMBB.robot_state[id].robot_pos.pos);
	Vec InfinityGoal(0,1000000); //Infinity Goal wird benutzt um den winkel zu bestimmen, attacken aus der mitte sind somit besser!
	Angle RtoBtoIG=AbsBallfromRobot.angle()-InfinityGoal.angle();
	double scaledangleresult=1000*(1-2*fabs(RtoBtoIG.get_rad_pi())/M_PI);
	//double scaledangleresult=RtoBtoIG.get_rad_pi();
	//cout<<"ScaledAngleresult no "<<id<<"  "<<scaledangleresult<<endl;
	ranking+=scaledangleresult;
	//**************************************************************************/
	
	
	//**************************************************************************/
	//Die Geschwindigkeit des Balles bezüglich des Roboters wird durch das Skalarprodukt der Ballposition mit der Ballgeschwindigkeit bestimmt.
	//Dieses Kriterium macht 500 Punkte +/- aus.
	Vec relativeBall=REMBB.robot_state[id].ball_pos.pos.toVec()-REMBB.robot_state[id].robot_pos.pos;
	Vec ballvelocity=REMBB.robot_state[id].ball_pos.velocity.toVec();
	if (ballvelocity.length()>1.0){
	double skalarprodukt=relativeBall.normalize() * ballvelocity.normalize();
	//cout <<"Skalarprodukt des Balles mit der Relativen Ballposition"<<skalarprodukt<<endl;
	double velbonus=-skalarprodukt*500;
        //cout <<"Ball Velocity Bonus "<<velbonus<<endl;

	ranking+=velbonus;
	}
	
	//**************************************************************************/
	
	
	
	
	//**************************************************************************/
	if (id==last_ball_owner)
	{
		ranking+=owner_bonus;
		owner_bonus-=10;
		if (owner_bonus<0)owner_bonus=0;
	}
	return ranking;
	//**************************************************************************/
	
};



