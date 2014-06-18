
#include "StaticPolicy.h"
#include "../States/RemoteBlackboard.h"
#include "../../../Structures/Journal.h"
#include <algorithm>
using namespace TribotsTools;
using namespace Tribots;
using namespace std;

StaticPolicy::StaticPolicy (unsigned int n, const ConfigReader& config) 
  : max_num_players(n), roles(n,n), robot_activation_time (REMBB.robot_state.size()) 
{
  prefs.readPreferences(config);
}


const char* StaticPolicy::get_name () const throw () {
  return policy_name.c_str();
}

    
void StaticPolicy::observe () throw () {
  for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
    if (!(REMBB.robot_state[i].playertype==playertype && REMBB.robot_state[i].in_game)) {
      robot_activation_time[i].update();
    }
  }
}


void StaticPolicy::update () throw () {
  try{

  // aktive Spieler erfassen
  unsigned int num_robots = REMBB.robot_state.size();
  unsigned int num_active_players=0;
  vector<bool> is_active_player (num_robots);
  for (unsigned int i=0; i<num_robots; i++) {
//    cerr << "i: " << i << " Robot" << REMBB.robot_state[i].id << ": " << REMBB.robot_state[i].playerrole << "/" 
//    << REMBB.robot_state[i].desired_playerrole << " active: " << (REMBB.robot_state[i].in_game && !REMBB.robot_state[i].comm_interrupted) << endl;
    if (REMBB.robot_state[i].playertype==playertype && REMBB.robot_state[i].in_game && !REMBB.robot_state[i].comm_interrupted) {
      if (REMBB.robot_state[i].playerrole != REMBB.robot_state[i].desired_playerrole) {
        cerr << "StaticPolicy: Ein anderer Wechsel ist noch nicht ganz abgeschlossen. Mache nix." << endl;
        return;
      }
      num_active_players++;
      is_active_player[i]=true;
    } else {
      is_active_player[i]=false;
    }
  }    
  if (num_active_players>max_num_players)
    num_active_players=max_num_players;

  
  // Soll-Rollenverteilung erfassen
  vector<string> target_roles (num_active_players);
  for (unsigned int i=0; i<num_active_players; i++)
    target_roles[i] = roles(num_active_players-1, i);

  
  // aus der target_roles alle Rollen loeschen, die durch neu aktivierte Spieler belegt sind
  Time now;
  for (unsigned int i=0; i<num_robots; i++)
    if (is_active_player[i] && now.diff_msec(robot_activation_time[i])<1000) {
      vector<string>::iterator it = find (target_roles.begin(), target_roles.end(), REMBB.robot_state[i].playerrole);
      if (it!=target_roles.end()) {
//        cerr << "F: " << REMBB.robot_state[i].id << "<->" << *it << "   ";
        target_roles.erase (it); 
      }
      is_active_player[i]=false;
      num_active_players--;
    }

  
  // aus der target_roles Liste alle Rollen loeschen, die von aktiven Playern belegt sind
  for (unsigned int i=0; i<num_robots; i++)
    if (is_active_player[i]) {
      vector<string>::iterator it = find (target_roles.begin(), target_roles.end(), REMBB.robot_state[i].playerrole);
      if (it!=target_roles.end()) {
//                cerr << "G: " << REMBB.robot_state[i].id << "<->" << *it << "   ";
	target_roles.erase (it);
	is_active_player[i]=false;
	num_active_players--;
      }
    }

  // Wenn Rollen uebriggeblieben sind, diese vergeben
  if (target_roles.size()>0 && num_active_players>0) {
    // Roboter nach ihren Positionen von hinten nach vorne ordnen
    unsigned int in=0;
    vector<pair<unsigned int,double> > indeces (num_active_players);
    for (unsigned int i=0; i<num_robots; i++) {
      if (is_active_player[i]) {
        cerr << "Index: " << i << endl;
	indeces[in].first=i;
	indeces[in].second=REMBB.robot_state[i].own_half*REMBB.robot_state[i].robot_pos.pos.y;
	in++;
      }
    }
    for (unsigned int i=0; i<indeces.size(); i++) // Blasen-Sortierung
      for (unsigned int j=1; j<indeces.size(); j++)
	if (indeces[j].second<indeces[j-1].second) {
	  unsigned int swi=indeces[j].first;
	  double swy=indeces[j].second;
	  indeces[j].first=indeces[j-1].first;
	  indeces[j].second=indeces[j-1].second;
	  indeces[j-1].first=swi;
	  indeces[j-1].second=swy;
	}
    
    for (unsigned int i=0; i<indeces.size() && num_active_players>0; i++) {
      REMBB.robot_state[indeces[i].first].desired_playerrole  = target_roles[0];
//      cerr << "O: "<<  REMBB.robot_state[indeces[i].first].id << "<->" << REMBB.robot_state[indeces[i].first].playerrole << "   ";
      cerr << "Automatische Rollenzuweisung: " << REMBB.robot_state[indeces[i].first].name << " bekommt Rolle " << target_roles[0] <<'\n';
      target_roles.erase (target_roles.begin());
      is_active_player[indeces[i].first]=false;
      num_active_players--;	
    }
  }
  else if ((REMBB.team_state.refstate == preOwnKickOff ||
            REMBB.team_state.refstate == preOwnGoalKick ||
            REMBB.team_state.refstate == preOwnCornerKick ||
            REMBB.team_state.refstate == preOwnThrowIn ||
            REMBB.team_state.refstate == preOwnFreeKick ||
            REMBB.team_state.refstate == preOwnPenalty && num_active_players == 0) && // alle spieler haben eine rolle, alle rollen belegt
            target_roles.size() == 0) { // Der folgende Code ist neu und bisher noch wenig getestet (April 08)
    map<int, string> alloc;
    map<int, int> idToIndex;
    for (unsigned int i=0; i<num_robots; i++) {
      if (REMBB.robot_state[i].playertype==playertype && REMBB.robot_state[i].in_game && !REMBB.robot_state[i].comm_interrupted) {
        if (REMBB.robot_state[i].playerrole != REMBB.robot_state[i].desired_playerrole) {
          cerr << "Wechsel auf bevorzugte Position: Bereits ein anderer Wechsel aktiv. Mache nix." << endl;
          return;
        }
        alloc[REMBB.robot_state[i].id] = REMBB.robot_state[i].playerrole;
        idToIndex[REMBB.robot_state[i].id] = i;
      }
    }
    if (alloc.size() > 1) {  // nur, wenn auch ein spieler da ist
      int violations, changes;
      map<int, string> targetAlloc = RoleAllocationSolver::solve(prefs, alloc, &violations, &changes);
      if (changes > 0) {
        cerr << "Wechsel auf bevorzugte Position wird durchgefuehrt. Violations:  " << violations << " Changes: " << changes << endl;
        cerr << "Aenderungen: ";
        for (map<int, string>::iterator i = targetAlloc.begin(); i != targetAlloc.end(); i++) {
          // DEBUG Check: if (idToIndex[i->first] < 0 || idToIndex[i->first] > num_robots) { cerr << "MIST" << endl; }
          cerr << REMBB.robot_state[idToIndex[i->first]].playerrole << "->" << i->second << " ";
          REMBB.robot_state[idToIndex[i->first]].desired_playerrole = i->second;
        }
        cerr << endl;
      }
    }
  }
  }catch(exception& e) {
    JERROR (e.what());
  }
}
