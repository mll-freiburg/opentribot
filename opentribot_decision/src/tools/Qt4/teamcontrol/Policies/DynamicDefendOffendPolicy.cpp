
#include "DynamicDefendOffendPolicy.h"
#include "../States/RemoteBlackboard.h"

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

DynamicDefendOffendPolicy::DynamicDefendOffendPolicy (Policy* def, Policy* off) : defensive_policy (def), offensive_policy (off) {
  was_offensive = false;
}

DynamicDefendOffendPolicy::~DynamicDefendOffendPolicy () throw () {
  delete defensive_policy;
  delete offensive_policy;
}

const char* DynamicDefendOffendPolicy::get_name () const throw () {
  return policy_name.c_str();
}

void DynamicDefendOffendPolicy::update () throw () {
  // Ballposition in Erfahrung bringen
  bool ball_found=false;
  unsigned int nearest_robot=99999;
  double smallest_dist=1e99;
  unsigned int num_robots=REMBB.robot_state.size();
  for (unsigned int sender=0; sender<num_robots; sender++)
    if (REMBB.robot_state[sender].in_game)
      if (REMBB.robot_state[sender].ball_pos.pos_known==BallLocation::known) {
	double len = (REMBB.robot_state[sender].ball_pos.pos-REMBB.robot_state[sender].robot_pos.pos).length();
	if (len<smallest_dist) {
	  smallest_dist=len;
	  nearest_robot=sender;
	  ball_found=true;
	}
      }
  if (ball_found) {
    Vec ball = REMBB.robot_state[nearest_robot].ball_pos.pos.toVec();
    if (REMBB.robot_state[nearest_robot].own_half!=REMBB.team_state.own_half)
      ball*=-1;
    if (REMBB.team_state.own_half*ball.y<-200 && was_offensive) {
      was_offensive=false;
      cerr << "schalte auf defensive Strategie um\n";
    } else if (REMBB.team_state.own_half*ball.y>200 && !was_offensive) {
      was_offensive=true;
      cerr << "schalte auf offensive Strategie um\n";
    }
  }

  if (was_offensive) {
    offensive_policy->update();
    defensive_policy->observe();
  } else {
    defensive_policy->update();
    offensive_policy->observe();
  }
}

