// 
// File:   BNewDribbleTest.h
// Author: stephangabler
//
// Created on 9. April 2008, 14:11
//

#ifndef _BNewDribbleTest_H
#define	_BNewDribbleTest_H

#include "../../../Fundamental/ConfigReader.h"
#include "../../../Player/BehaviorPlayer.h"
#include "../../../Fundamental/geometry.h"
#include "../../Skills/BallHandling/SDribbleBallToPosRL.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include <cmath>
#include <vector>



namespace Tribots {
  
  struct wayPoint {
    Vec pos;
    XYRectangle area;
  };


	class BNewDribbleTest : public Tribots::Behavior
		{
		public:
			BNewDribbleTest(double transVel = 2.3);
			virtual ~BNewDribbleTest() throw();

			virtual bool	checkCommitmentCondition(const Time&) throw();
			virtual bool	checkInvocationCondition(const Time&) throw();
			virtual DriveVector getCmd(const Time&) throw(TribotsException);
			virtual void	gainControl(const Time&) throw(TribotsException);
			virtual void	loseControl(const Time&) throw(TribotsException);  
			// virtual void	updateTactics (const TacticsBoard& tb) throw ();
			
			bool set_field_area(std::string area);
			XYRectangle* get_dribble_area();
			Vec* get_dribble_area_center();

		protected:
			double		dribblevel;
			bool			leftArea;
			Vec				offset;
			SDribbleBallToPosRL*	skill;

			int						timer;
			RobotLocation	robot;
			FieldGeometry fgeom;
			std::vector<wayPoint> wayPoints;
			int						currentWayPoint;

			void addWayPoint(Vec);
			void print_way_points();

		};
}



#endif	/* _BNewDribbleTest_H */

