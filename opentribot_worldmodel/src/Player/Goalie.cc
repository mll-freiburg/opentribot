
#include "Goalie.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "../Behavior/Behaviors/Goalie/BGoalieAttackBall.h"
//#include "../Behavior/Behaviors/Goalie/BGoaliePenaltyInterceptBall.h"
//#include "../Behavior/Behaviors/Goalie/BGoaliePenaltyAttackBall.h"
//#include "../Behavior/Behaviors/Goalie/BGoaliePenaltyRaised.h"
#include "../Behavior/Behaviors/Goalie/BGoalieBallInGoal.h"
#include "../Behavior/Behaviors/Goalie/BGoalieFetchBall.h"
#include "../Behavior/Behaviors/Goalie/BGoalieFetchBallNearGoalPost.h"
#include "../Behavior/Behaviors/Goalie/BGoalieFetchBallLaterally.h"
#include "../Behavior/Behaviors/Goalie/BGoalieRaisedBall.h"
#include "../Behavior/Behaviors/Goalie/BGoalieGetAwayFromGoalPosts.h"
#include "../Behavior/Behaviors/Goalie/BGoalieGetOutOfGoal.h"
#include "../Behavior/Behaviors/Goalie/BGoalieGoalKick.h"
#include "../Behavior/Behaviors/Goalie/BGoaliePatrol.h"
#include "../Behavior/Behaviors/Goalie/BGoaliePositioning.h"
#include "../Behavior/Behaviors/Goalie/BGoalieBaselinePositioning.h"
//#include "../Behavior/Behaviors/Goalie/BGoaliePrePenalty.h"
#include "../Behavior/Behaviors/Goalie/BGoaliePenalty.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/SpecialGameStates/BTestStateStop.h"
#include "../Behavior/Behaviors/Goalie/BGoalieFastPositioning.h"
#include "../Behavior/Behaviors/Goalie/BGoalieOpponentGoalKick.h"
#include "../Behavior/Behaviors/Goalie/BGoalieOpponentKickOff.h"
#include "../Behavior/Behaviors/Goalie/BGoaliePositioningChipKick.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("Goalie"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,std::bad_alloc) {
      return new Goalie (reader);
    }
  };
  Builder the_builder;
}



Goalie::Goalie (const ConfigReader& cfg) throw () : BehaviorPlayer ("Goalie") {
  const FieldGeometry& fg (MWM.get_field_geometry());
  const RobotProperties& rp (MWM.get_robot_properties());

  bool consider_obstacles = true;
  cfg.get ("Goalie::consider_obstacles", consider_obstacles);
  vector<double> home_yy;
  cfg.get ("Goalie::home", home_yy);
  if (home_yy.size()==0)
    home_yy.push_back (500);
  if (home_yy.size()==1)
    home_yy.push_back (home_yy[0]);
  double mindist_y=200;
  cfg.get ("Goalie::mindist", mindist_y);
  Vec near_home_pos (0, -0.5*fg.field_length+home_yy[0]);
  Vec far_home_pos (0, -0.5*fg.field_length+home_yy[1]);
  Vec patrol_home_pos (0, -0.5*fg.field_length+home_yy[1]);

  if (mindist_y>home_yy[1]-100)
    mindist_y=home_yy[1]-100;
  Vec right_end (0.5*fg.goal_width, -0.5*fg.field_length+mindist_y);
  bool has_goalie_wings = (rp.robot_width>550);
  Angle max_heading = (has_goalie_wings ? Angle::deg_angle (40) : Angle::deg_angle (60));
  double attack_area = 1500;
  cfg.get ("Goalie::attack_area", attack_area);
  Vec attack_area1 (-attack_area, -0.5*fg.field_length);
  Vec attack_area2 (attack_area, -0.5*fg.field_length+attack_area);
  bool kick_permission=true;
  cfg.get ("Goalie::kick_permission", kick_permission);
  double fetching_area_width = 3500;
  cfg.get ("Goalie::fetching_area_width", fetching_area_width);
  bool use_comm_ball = true;
  cfg.get ("Goalie::use_comm_ball", use_comm_ball);
  double home_penalty = 200;
  cfg.get ("Goalie::home_penalty", home_penalty);
  bool use_opp_gk_positioning = false;
  cfg.get ("Goalie::opponent_goal_kick_active", use_opp_gk_positioning);
  double opp_gk_offset_x = 0;
  double opp_gk_offset_y = home_yy[0];
  vector<double> gk_offsets;
  if (cfg.get ("Goalie::opponent_goal_kick_offset", gk_offsets)>0) {
    opp_gk_offset_x = gk_offsets [0];
    opp_gk_offset_y = 0;
    if (gk_offsets.size()>=2) 
      opp_gk_offset_y = gk_offsets [1];
  }
  double penalty_max_excenter=0;
  cfg.get ("Goalie::penalty_excenter", penalty_max_excenter);

//  vector<double> prob_raised;
//  cfg.get ("Goalie::raised_probabilities", prob_raised);
//  if (prob_raised.size()<3) {
//    // default, wenn cfg keine sachgerechte Information enthaelt
//    prob_raised.resize (3);
//    prob_raised[0]=0.4;
//    prob_raised[1]=0.2;
//    prob_raised[2]=0.4;
//  }

  addOption (new BGameStopped);
  addOption (new BTestStateStop);
  addOption (new BGoaliePenalty);
  addOption (new BGoalieBallInGoal);
  addOption (new BGoalieGetAwayFromGoalPosts);
//  addOption (new BGoaliePrePenalty (home_penalty, penalty_max_excenter));
//  addOption (new BGoaliePenaltyInterceptBall ());
  addOption (new BGoalieOpponentGoalKick (opp_gk_offset_x, opp_gk_offset_y, use_opp_gk_positioning));
  addOption (new BGoalieOpponentKickOff (false));
//  addOption (new BGoaliePenaltyRaised (prob_raised[0], prob_raised[1], prob_raised[2]));
//  addOption (new BGoalieGoalKick (max_heading, has_goalie_wings, kick_permission));  // wird z.Z. nicht benoetigt, da GoalKick und FreeKick stets ausserhalb des Strafraums ausgefuehrt werden (Regeln 2005)
//  addOption (new BGoalieBaselinePositioning (use_comm_ball));
  addOption (new BGoaliePositioningChipKick (&goto_pos_skill));
  addOption (new BGoalieFastPositioning (far_home_pos, right_end, max_heading, use_comm_ball));
  addOption (new BGoalieRaisedBall);
  addOption (new BGoalieFetchBallNearGoalPost);
  addOption (new BGoalieFetchBallLaterally (0.5*(fetching_area_width-fg.goal_width), max_heading, has_goalie_wings));
//  addOption (new BGoaliePenaltyAttackBall (consider_obstacles, attack_area1, attack_area2+Vec(0,1000), kick_permission));
  addOption (new BGoalieAttackBall (consider_obstacles, attack_area1, attack_area2, kick_permission, &goto_pos_skill));
  addOption (new BGoalieFetchBall (0.5*(fetching_area_width-fg.goal_width)));
  addOption (new BGoalieGetOutOfGoal (0.5*mindist_y));
  addOption (new BGoaliePositioningFarBall (far_home_pos, right_end, max_heading, use_comm_ball, &goto_pos_skill));
  addOption (new BGoaliePositioning (near_home_pos, right_end, max_heading, use_comm_ball, &goto_pos_skill));
  addOption (new BGoaliePatrol (patrol_home_pos, &goto_pos_skill));
}

Goalie::~Goalie () throw () {;}

DriveVector Goalie::process_drive_vector (Time texec) throw () {
  texec.add_msec(50);  // dann wird das Ueberschwingen geringer
  return BehaviorPlayer::process_drive_vector(texec);
}
