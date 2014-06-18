#include "BallLocationHysteresis.h"
#include "../WorldModel/WorldModel.h"

namespace Tribots {
	BallLocationHysteresis::BallLocationHysteresis(int margin) {
		const FieldGeometry& field = MWM.get_field_geometry();
		const double& length = field.field_length;
		const double& width = field.field_width;

		vArea = MIDDLE;
		hArea = CENTER;

		areas.resize(10);

		// define 4 independent divisions of the field
		areas[LEFT] = XYRectangle(
			Vec(-width/2 - margin, -length/2 - margin), 
			Vec(-width/6 + margin, length/2 + margin));

		areas[CENTER] = XYRectangle(
			Vec(-width/6 - margin, -length/2 - margin), 
			Vec(width/6 + margin, length/2 + margin));

		areas[RIGHT] = XYRectangle(
			Vec(width/6 - margin, -length/2 - margin),
			Vec(width/2 + margin, length/2 + margin));

		areas[LEFTHALF] = XYRectangle(
			Vec(-width/2 - margin, -length/2 - margin), 
			Vec(0 + margin, length/2 + margin));

		areas[RIGHTHALF] = XYRectangle(
			Vec(0 - margin, -length/2 - margin),
			Vec(width/2 + margin, length/2 + margin));

		areas[FRONT] = XYRectangle(
			Vec(-width/2 - margin, length/4 - margin),
			Vec(width/2 + margin, length/2 + margin));

		areas[MIDDLE] = XYRectangle(
			Vec(-width/2 - margin, -length/4 - margin),
			Vec(width/2 + margin, length/4 + margin));

		areas[BACK] = XYRectangle(
			Vec(-width/2 - margin, -length/2 - margin),
			Vec(width/2 + margin, -length/4 + margin));

		areas[FRONTHALF] = XYRectangle(
			Vec(-width/2 - margin, 0 - margin),
			Vec(width/2 + margin, length/2 + margin));

		areas[BACKHALF] = XYRectangle(
			Vec(-width/2 - margin, -length/2 - margin),
			Vec(width/2 + margin, 0 + margin));
	}



	void BallLocationHysteresis::update(const Time& time) {
		const BallLocation& ball = MWM.get_ball_location(time);

		if(ball.pos_known == BallLocation::known || 
		  	ball.pos_known == BallLocation::communicated) {
			Vec ball_pos = ball.pos.toVec();

			// if the ball is outside any area, the currently active area is assumed
			if(!areas[hArea].is_inside(ball_pos)) {
				if(areas[LEFT].is_inside(ball_pos))
					hArea = LEFT;
				if(areas[CENTER].is_inside(ball_pos))
					hArea = CENTER;
				if(areas[RIGHT].is_inside(ball_pos))
					hArea = RIGHT;
			}

			if(!areas[vArea].is_inside(ball_pos)) {
				if(areas[FRONT].is_inside(ball_pos))
					vArea = FRONT;
				if(areas[MIDDLE].is_inside(ball_pos))
					vArea = MIDDLE;
				if(areas[BACK].is_inside(ball_pos))
					vArea = BACK;
			}

			if(!areas[hAreaHalf].is_inside(ball_pos)) {
				if(areas[LEFTHALF].is_inside(ball_pos))
					hAreaHalf = LEFTHALF;
				if(areas[RIGHTHALF].is_inside(ball_pos))
					hAreaHalf = RIGHTHALF;
			}

			if(!areas[vAreaHalf].is_inside(ball_pos)) {
				if(areas[FRONTHALF].is_inside(ball_pos))
					vAreaHalf = FRONTHALF;
				if(areas[BACKHALF].is_inside(ball_pos))
					vAreaHalf = BACKHALF;
			}
		}
	}
}
