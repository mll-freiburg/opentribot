#include "BPreOwnKickOff.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"

namespace Tribots {
	BPreOwnKickOff::BPreOwnKickOff() throw() : BStandardSituation("BOwnKickOff") {
		executionFinished = false;
	}



	bool BPreOwnKickOff::checkInvocationCondition(const Time& time) throw () {
		return MWM.get_game_state().refstate == preOwnKickOff;
	}



	bool BPreOwnKickOff::checkCommitmentCondition(const Time& time) throw() {
		if(executionFinished)
			return false;
		return 
			MWM.get_game_state().refstate == preOwnKickOff ||
			MWM.get_game_state().refstate == freePlay;
	}



	void BPreOwnKickOff::gainControl(const Time& time) throw() {
		executionFinished = false;
	}



	DriveVector BPreOwnKickOff::getCmdBefore(const Time& time) throw() {
		const FieldGeometry& field = MWM.get_field_geometry();

		SPhysGotoPosAvoidObstacles goTo;
		
		vBallPosMem = MWM.get_ball_location(time).pos.toVec();
		vRobotPosMem = MWM.get_robot_location(time).pos;

		Vec& vBallPos = vBallPosMem;
		Vec& vRobotPos = vRobotPosMem;

		Vec vTargetAbsolute = vRobotPos;
		Vec vTargetRelative = vRobotPos - vBallPos;
		Vec vLookingDir;

		switch(WBOARD->getStandardSituationRole()) {
			// 1st ROBOT, getting the ball
			case WhiteBoard::STANDARD_ROLE_A:
			case WhiteBoard::STANDARD_ROLE_AB:
			case WhiteBoard::STANDARD_ROLE_ABCDE:
				vTargetRelative.x = -500;
				vTargetRelative.y = -500;
				break;

			// 2nd ROBOT, touching the ball
			case WhiteBoard::STANDARD_ROLE_C:
			case WhiteBoard::STANDARD_ROLE_CDE:
				vTargetRelative.x = 500;
				vTargetRelative.y = 0;
				vLookingDir.x = 0;
				vLookingDir.y = 1;
				break;

			// 3rd ROBOT, safety
			case WhiteBoard::STANDARD_ROLE_E:
			case WhiteBoard::STANDARD_ROLE_DE:
				vTargetAbsolute.x = 0;
				vTargetAbsolute.y = -field.field_length / 2 + 2000;
				break;

			// 4th ROBOT
			case WhiteBoard::STANDARD_ROLE_B:
				vTargetRelative.x = 0;
				vTargetRelative.y = -2000;
				break;

			// 5th ROBOT
			case WhiteBoard::STANDARD_ROLE_D:
				vTargetAbsolute.x = -field.field_width / 4;
				vTargetAbsolute.y = -field.field_length / 4;
				break;
		}

		vTargetAbsolute = vBallPos + vTargetRelative + vTargetAbsolute - vRobotPos;

		if(vLookingDir.length() < 0.1 || (vRobotPos - vTargetAbsolute).length() > 500)
			vLookingDir = vBallPos - vRobotPos;

		goTo.init(vTargetAbsolute, vLookingDir, true);
		return goTo.getCmd(time);
	}
	
	

	DriveVector BPreOwnKickOff::getCmdAfter(const Time& time) throw() {
		SPhysGotoPos goTo;
		SApproachMovingBall approachBall;
			
		const BallLocation& ball = MWM.get_ball_location(time);

		Vec vBallPos = ball.pos.toVec();
		Vec vRobotPos = MWM.get_robot_location(time).pos;
		Vec vLookingDir = vBallPos - vRobotPos;
		Vec vTargetAbsolute;

		switch(WBOARD->getStandardSituationRole()) {
			// 1st ROBOT, alone
			case WhiteBoard::STANDARD_ROLE_ABCDE:
				// just give up control
				executionFinished = true;
				break;

			// 2nd ROBOT, touches the ball
			case WhiteBoard::STANDARD_ROLE_C:
			case WhiteBoard::STANDARD_ROLE_CDE:
				vLookingDir.x = 0;
				vLookingDir.y = 1;

				if(getExecutionPhase() > 1) {
					// remove from the ball
					vTargetAbsolute = vRobotPosMem;

					// commitment condition
					if((vRobotPos - vTargetAbsolute).length() < 300 || getExecutionTime(time) > 1000)
						executionFinished = true;
				} else {
					// touch the ball
					vTargetAbsolute = vBallPosMem + 200 * (vBallPosMem - vRobotPosMem).normalize();

					if((vRobotPos - vTargetAbsolute).length() < 300 
							&& ball.velocity.toVec().length() > 0.2)
						setExecutionPhase(2);
				}

				goTo.init(vTargetAbsolute, vLookingDir, 1.0, false, false);
				return goTo.getCmd(time);

			// all other robots
			default:
				// wait until phase two
				if(getExecutionPhase() > 1)
					executionFinished = true;
		}

		return DriveVector();
	}
}
