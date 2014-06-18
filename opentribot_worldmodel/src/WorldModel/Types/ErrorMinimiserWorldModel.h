
#ifndef _Tribots_ErrorMinimiserWorldModel_h_
#define _Tribots_ErrorMinimiserWorldModel_h_

#include "WorldModelTypeBase.h"
#include "../SL/SelfLocalisation.h"
#include "../Ball/MultiCameraBallFilter.h"
#include "../Obstacles/EMAObstacleFilter.h"


namespace Tribots {

  /** Weltmodell, das auf einer Fehlerminimierung und Kalmanfilter beruht */
  class ErrorMinimiserWorldModel : public WorldModelTypeBase {
  private:
    SelfLocalisation* sl;                       // Selbstlokalisierung
    MultiCameraBallFilter ball_filter;  // Ballfilter (Position, Geschwindigkeit, Attribute)
    EMAObstacleFilter obstacle_filter;           // Hindernisfilter (Position, Attribute)
    Time latest_vis_timestamp;                   // Zeitstempel des letzten Bildes
    Time latest_slupdate_timestamp;                // Zeitstempel des letzten SL-Updates
    bool cycle_delayed;    // War der letzte Zyklus verspaetet?

    unsigned long int usec_sl, usec_ball, usec_obstacle, num_cycle;  ///< zum Zeitloggen
    bool report_computation_time;
    bool report_computation_time_per_cycle;

  public:
    ErrorMinimiserWorldModel (const ConfigReader&) throw (std::bad_alloc);
    ~ErrorMinimiserWorldModel () throw ();

    RobotLocation get_robot (Time) const throw ();
    BallLocation get_ball (Time) const throw ();
    ObstacleLocation get_obstacles (Time) const throw ();
    Time get_timestamp_latest_update () const throw ();
    unsigned int update_localisation () throw ();
    void reset () throw ();
    void reset (const Vec) throw ();
    void reset (const Vec, const Angle) throw ();
    void slMirrorHint (Vec) throw ();
  };
}

#endif
