/*
 *  RolePreferences.cpp
 *
 *  Created by Sascha Lange on 31.03.08.
 *  Copyright 2008 University of Osnabrueck. All rights reserved.
 *
 */

#include "RolePreferences.h"
#include "../Structures/TribotsException.h"
#include <sstream>

using namespace Tribots;
using namespace std;

RolePreferences::RolePreferences()
{};

void 
RolePreferences::readPreferences(const ConfigReader& config, const string section)
{
  if (config.get((section+"::pref_roles").c_str(), description.roles) <= 0) {
    throw InvalidConfigurationException("pref_roles");
  }
  if (config.get((section+"::pref_robots").c_str(), description.robots) <= 0) {
    throw InvalidConfigurationException("pref_robots");
  }
  for (vector<int>::iterator r = description.robots.begin(); r != description.robots.end(); r++) {
    stringstream stream;
    vector<string> roles;
    stream << section << "::pref_robot" << *r;
    if (config.get(stream.str().c_str(), roles) < 0) {
      throw InvalidConfigurationException(stream.str().c_str());
    }
/*    if (roles.size() == 0) {
      roles = description.roles;
    }*/
    preferences[*r] = roles;
  }
}

const vector<string>& RolePreferences::operator[] (int robot) const
{
  return const_cast<RolePreferences*>(this)->preferences[robot];
}
vector<string>& RolePreferences::operator[] (int robot)
{
  return preferences[robot];
}
vector<string>& RolePreferences::operator[] (const std::string& robot)
{
  stringstream str(robot);
  int n;
  str >> n;
  return (*this)[n];
}
const vector<string>& RolePreferences::operator[] (const std::string& robot) const
{
  stringstream str(robot);
  int n; 
  str >> n;
  return (*this) [n];
}

bool RolePreferences::contains(int robot, const string& role) const
{
  const vector<string>& prefs = (*this)[robot];
  for (unsigned int i=0; i < prefs.size(); i++)
    if (prefs[i] == role) return true;
  return false;
}

static void calcValueOfAlloc(const RolePreferences& prefs, 
                             const vector<int>& allocations, const vector<int>& robots, 
                             const vector<string>& roles, int* violations, int* changes)
{
  *violations = *changes = 0;
  for (unsigned int i=0; i < allocations.size(); i++) {
    if (allocations[i] != static_cast<int>(i)) *changes += 1;
    if (! prefs[robots[i]].size() == 0 && !prefs.contains(robots[i],roles[allocations[i]])) {
  //    cerr << robots[i] << ": " << roles[allocations[i]] << endl; 
      *violations += 1;
    }
  }
}

const map<int, string>
RoleAllocationSolver::solve(const RolePreferences& prefs, const map<int, string> presentAlloc, int* rviolations, int* rchanges)
{
  if (presentAlloc.size() <= 1) return presentAlloc; // nix umzuordnen
  vector<string> roles;
  vector<int> robots;
  for (map<int, string>::const_iterator i = presentAlloc.begin(); i != presentAlloc.end(); i++) {
    roles.push_back(i->second);  // present allocation in indices is always:    1,2,3,4,5,...
    robots.push_back(i->first); 
  }
  
  vector<int> allocation(roles.size());
  vector<int> used(roles.size());
  vector<int> bestAllocation(roles.size());
  int bestViolations = roles.size() + 1;
  int bestChanges = roles.size() + 1;
  int pos = 0; 
  for (unsigned int i=0; i < allocation.size(); i++) { allocation[i] = -1; }
  
  do {
    int f = allocation[pos];
    do {
      f++;
    } while (f < static_cast<int>(used.size()) && used[f]);
    if (f >= static_cast<int>(used.size())) {
      if (allocation[pos] >= 0) used[allocation[pos]] = 0; // wenn der schon zugewiesen war
      allocation[pos] = -1;
      pos--;
    }
    else {
      if (allocation[pos] >= 0) used[allocation[pos]] = 0;
      used[f] = 1;
      allocation[pos] = f;
      pos++;
    }
    if (pos >= static_cast<int>(allocation.size())) {
      int changes=0, violations=0;
      calcValueOfAlloc(prefs, allocation, robots, roles, &violations, &changes);
      if (bestViolations > violations || (bestViolations == violations && bestChanges > changes)) {
        bestAllocation = allocation;
        bestViolations = violations; bestChanges = changes;
      }
      pos--;
    }
  } while (pos >= 0);
  map<int, string> solution;
  for (unsigned int i=0; i < bestAllocation.size(); i++) {
    solution[robots[i]] = roles[bestAllocation[i]];
  }
  if (rviolations != 0) *rviolations = bestViolations;
  if (rchanges != 0) *rchanges = bestChanges;
  return solution;
}


#if 0

int main(int argc, char* argv[])
{
  if (argc != 2) { cerr << "Aufruf: " << argv[0] << " config_file.cfg" << endl; exit(1); }
  ConfigReader config;
  config.append_from_file(argv[1]);
  RolePreferences prefs;
  prefs.readPreferences(config);
  
  for (unsigned int i=0; i < 10; i++) {
    cerr << "Robot " << i << ":";
    for (unsigned int p=0; p < prefs[i].size(); p++) {
      cerr << " " << prefs[i][p];
    }
    cerr << " bools:";
    for (unsigned int p=0; p < prefs.description.roles.size(); p++) {
      cerr << " " << (prefs.contains(i, prefs.description.roles[p]) ? "T" : "F");
    }
    cerr << endl;
  }
  cerr << "Roles:";
  for (unsigned int i=0; i < prefs.description.roles.size(); i++) {
    cerr << " " << prefs.description.roles[i];
  }
  cerr << endl << endl;
  
  map<int, string> allocation;
  allocation[5] = "left";
  allocation[4] = "right";
  allocation[9] = "ballL";
  allocation[3] = "ballR";
  allocation[7] = "safety";
  
  int violations, changes;
  cerr << "Present allocation: ";
  for (map<int, string>::const_iterator i = allocation.begin(); i != allocation.end(); i++) {
    cerr << i->first << ": " << i->second << " ";
  }
  cerr << endl;
  
  map<int, string> newAllocation = RoleAllocationSolver::solve(prefs, allocation, &violations, &changes);
  for (map<int, string>::const_iterator i = newAllocation.begin(); i != newAllocation.end(); i++) {
    cerr << i->first << ": " << i->second << " ";
  }
  cerr << endl << "Cost violations/changes: " << violations << "/" << changes << endl;
  
  return 0;
}

#endif
