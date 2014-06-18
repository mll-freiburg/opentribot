#ifndef _TRIBOTS_BALLLOCATIONHYSTERESIS_H_
#define _TRIBOTS_BALLLOCATIONHYSTERESIS_H_

#include "geometry.h"
#include "Time.h"
#include <vector>

namespace Tribots {
	class BallLocationHysteresis {
		public:
			BallLocationHysteresis(int margin = 500);

			// update the internal ball location with hysteresis
			void update(const Time& t);

			// partitioning along the width of the field
			inline bool isLeft() const { return hArea == LEFT; }
			inline bool isCenter() const { return hArea == RIGHT; }
			inline bool isRight() const { return hArea == RIGHT; }

			inline bool isLeftHalf() const { return hAreaHalf == LEFTHALF; }
			inline bool isRightHalf() const { return hAreaHalf == RIGHTHALF; }

			// partitioning along the length of the field
			inline bool isFront() const { return vArea == FRONT; }
			inline bool isMiddle() const { return vArea == MIDDLE; }
			inline bool isBack() const { return vArea == BACK; }

			inline bool isFrontHalf() const { return vAreaHalf == FRONTHALF; }
			inline bool isBackHalf() const { return vAreaHalf == BACKHALF; }


		private:
			// area names for convenient access to areas
			enum AreaId {
				LEFT = 0,
				CENTER = 1,
				RIGHT = 2,
				FRONT = 3,
				MIDDLE = 4,
				BACK = 5,
				LEFTHALF = 6,
				RIGHTHALF = 7,
				FRONTHALF = 8,
				BACKHALF = 9,
			};

			// container storing some default areas
			std::vector<XYRectangle> areas;

			// currently active areas
			AreaId hArea;
			AreaId vArea;
			AreaId hAreaHalf;
			AreaId vAreaHalf;
	};
}

#endif
