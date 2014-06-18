// Copyright (C) 2009 Rosen Diankov (rdiankov@cs.cmu.edu)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <ros/master.h>
#include <boost/shared_ptr.hpp>

#include <eposdrivers/eposcontroller.h>

#include <unistd.h>
#include <signal.h>

using namespace std;
using namespace OpenRAVE;

void sigint_handler(int);
boost::shared_ptr<openrave_robot_control::OpenRAVEController> s_pcontroller;

int main(int argc, char ** argv)
{
    signal(SIGINT,sigint_handler); // control C
        
    dReal fMaxVelMult = 1;

    // parse the command line options
    string robotname,manipname="arm";
    int canport = 0;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            // print the arguments and exit
            printf("robotlinks_filter_node [--can port] [--robotfile openravefile] [--manipname name] [--maxvelmult multiplier]\n"
                   "  Start a node to control the EPOS device and publish ROS interfaces.\n"
                   "  Currently the robot file specified has to be in OpenRAVE format\n");
            return 0;
        }
        if( strcmp(argv[i], "--robotfile") == 0 ) {
            robotname = argv[i+1];
            i += 2;
        }
        if( strcmp(argv[i], "--manipname") == 0 ) {
            manipname = argv[i+1];
            i += 2;
        }
        else if( strcmp(argv[i], "--can") == 0 ) {
            canport = atoi(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--maxvelmult") == 0 ) {
            fMaxVelMult = atof(argv[i+1]);
            i += 2;
        }
        else
            break;
    }

    ros::init(argc,argv,"epos_server", ros::init_options::NoSigintHandler);

    if( !ros::master::check() )
        return 1;
    
    s_pcontroller.reset(new EPOSController(canport, robotname, manipname, fMaxVelMult));
    s_pcontroller->init();
    ros::spin();

    s_pcontroller.reset();
    return 0;
}

void sigint_handler(int)
{
    s_pcontroller->shutdown();
    ros::shutdown();
}
