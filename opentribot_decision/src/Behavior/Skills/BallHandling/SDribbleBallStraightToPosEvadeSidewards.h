#ifndef _TRIBOTS_SDRIBBLEBALLSTRAIGHTTOPOSEVADESIDEWARDS_H_
#define _TRIBOTS_SDRIBBLEBALLSTRAIGHTTOPOSEVADESIDEWARDS_H_

#include "../../Skill.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/RobotLocation.h"
#include "../../../Structures/RobotProperties.h"
#include "../../../Structures/FieldGeometry.h"
#include "../../../Structures/ObstacleLocation.h"
#include "../../../Fundamental/PIDController.h"

namespace Tribots {
  
  //  \TODO: Dieses Verhalten beinhaltete eine Funktionalität, sich in der
  //         Zielnähe auf das Ziel auszurichten und rechtzeitig zu schießen.
  //         Diese Funktionalität muss vielleicht in ein Shoot-Behavior 
  //         übernommen werden.
  
  /**
   * Dribbelt den Ball auf einer geraden Linie zum Ziel und weicht dabei
   * Hindernissen durch eine Seitwärtsbewegung ohne sich vom Ziel wegzudrehen
   * aus. Funktioniert nur gut, wenn sich das Ziel innerhalb des Kegels +/- 45
   * Grad _vor_ dem Roboter befindet.
   */
  class SDribbleBallStraightToPosEvadeSidewards : public Skill
  {
  public:
    SDribbleBallStraightToPosEvadeSidewards(double safetyMargin = 800.) throw();
    virtual ~SDribbleBallStraightToPosEvadeSidewards() throw ();

    /**
     * sets the minimal distance which the robot should try to maintain
     * while passing obstacles.
     * \param margin margin in mm
     */
    void setSafetyMargin(double margin);
    /** 
     * sets the target point to approach and the point that the robot
     * should point at while approaching the target. Depending on the 
     * driving velocity the angle between target and pointing position
     * may differ to some extent without loosing the ball. If approachingGoal
     * is set to true, the skill considers the dimensions of the goal while 
     * choosing waypoints for evasive maneuvers. It tries to hit the goal in 
     * any case, even if the target is totally blocked by obstacles.
     * \attention The pointing direction is only a desired goal; the behavior may
     * chose to reduce the difference by choosing a pointing direction that is
     * closer to 
     * the driving direction.
     * \param target the position that the robot should approach
     * \param pointTo the position that the robot should point to
     * \param approachingGoal tells the skill whether or not it is used to 
     *        approach the opponent's goal
     */
    void setTarget(const Vec& target, const Vec& pointTo,
                   bool approachingGoal = false);
    /**
     * sets the desired transitional velocity.
     */
    void setTransVel(double transVel);
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    virtual void gainControl(const Time&) throw(TribotsException);
		
		virtual void updateTactics (const TacticsBoard&) throw ();
    
  protected:
    int tendency;    
    
    Vec target;
    Vec pointTo;
    bool approachingGoal;
    double transVel;

    PIDController headingController;

    ObstacleLocation const* abs_obstacles; 
    RobotLocation const* robot;
    RobotProperties const* robot_properties;
    FieldGeometry const* fgeom;
    Frame2d world2robot, robot2world;
    Frame2d world2quad, quad2world;
    Vec oppGoalPos;
    XYRectangle fieldarea;
    double safety_margin;
    Vec startpos;
    LineSegment target_segment;
    Quadrangle quad;
    Quadrangle behind_goalarea;
		
		bool saveInfight;  ///< whether or not the robot is allowed to open a shoot line for the opponent during infight situation
		Vec startingPositionOfLastManeuver;
		Time timeOfLastSwitch;

    void init_world_model_data(const Time& tt);
    Vec compute_next_position_to_approach(Vec abs_main_obstacle, double abs_main_obstacle_width, const Time& t);
  };
}


#endif
