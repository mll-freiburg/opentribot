
#ifndef _TribotsTools_ControlGUIState_h_
#define _TribotsTools_ControlGUIState_h_

#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/ObstacleLocation.h"
#include "../../Structures/VisibleObject.h"
#include "../../Structures/GameState.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Structures/MessageBoard.h"
#include <string>

namespace TribotsTools {

  struct ControlGUIState {
    std::string hostname;
    unsigned int port;
    bool comm_okay;
    bool comm_interrupted;
    Tribots::Time latest_receive;

    std::string playerrole;
    std::string playertype;
    Tribots::RefereeState refstate;
    bool in_game;
    Tribots::FieldGeometry field_geometry;

    Tribots::MessageBoard message_board;

    std::vector<std::string>list_players;
    std::vector<std::string>list_roles;

    bool desired_in_game;
    std::string desired_playertype;
    std::string desired_playerrole;
    Tribots::RefereeState desired_refstate;

    bool slhint_request;
    Tribots::Vec slhint_pos;
    Tribots::Angle slhint_angle;
    bool debug_image_request;

    std::vector<Tribots::VisibleObjectList> visible_objects;
    Tribots::BallLocation ball_pos;
    Tribots::RobotLocation robot_pos;
    Tribots::ObstacleLocation obs_pos;
    float motor_vcc;

    bool field_geometry_request;
    bool playertypelist_request;
    bool playerrolelist_request;
    bool first_receive;
    std::vector<bool> was_comm_okay;

    bool exitRequest;

    Tribots::Vec drive_to_pos;
    Tribots::Angle drive_to_angle;
    bool drive_to_request;
  };

}

#endif
