#ifndef _ROLEPREFERENCES_H_
#define _ROLEPREFERENCES_H_
/*
 *  PreferencedRoles.h
 *
 *  Created by Sascha Lange on 31.03.08.
 *  Copyright 2008 University of Osnabrueck. All rights reserved.
 *
 */

#include <map>
#include <vector>
#include <string>
#include "ConfigReader.h"

namespace Tribots {
  
  using std::vector;
  using std::string;
  using std::map;

  class RolePreferencesDescription {
  public:
    vector<string> roles;
    vector<int> robots;
  };

  class RolePreferences {
  protected:
    map<int, vector<string> > preferences;   ///< list of preferred roles for all robots.
  public:
    RolePreferencesDescription description; 
    
    RolePreferences();
    void readPreferences(const ConfigReader&, const string section="Coach");
    
    const vector<string>& operator[] (int) const; ///< get list of preferred roles for robot by number
    vector<string>& operator[] (int); ///< get writable list of preferred roles for robot by number
    vector<string>& operator[] (const std::string&); ///< convinience function to get list of preferred roles 
    const vector<string>& operator[] (const std::string&) const;  ///< convinience function to get list of preferred roles
//  const vector<string>& operator[] (const char*) const; ///< convinience function to get list of preferred roles

    bool contains(int robot, const string& role) const;
  };
  
  class RoleAllocationSolver {
  public:
    static const map<int, string> solve(const RolePreferences& prefs, const map<int, string> presentAlloc,
                                        int* violations=0, int* changes=0);
  };
  
}

#endif

