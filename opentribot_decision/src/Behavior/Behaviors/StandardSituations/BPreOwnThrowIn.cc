#include <cmath>
#include "BPreOwnThrowIn.h"
#include "../../Skills/ApproachingBall/SApproachMovingBall.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"

using std::max;

namespace Tribots {
	BPreOwnThrowIn::BPreOwnThrowIn() throw() : BStandardSituation("BOwnThrowIn") {
		executionFinished = false;
	}



	bool BPreOwnThrowIn::checkInvocationCondition(const Time& time) throw () {
		return MWM.get_game_state().refstate == preOwnThrowIn;
	}



	bool BPreOwnThrowIn::checkCommitmentCondition(const Time& time) throw() {
		if(executionFinished)
			return false;
		return 
			MWM.get_game_state().refstate == preOwnThrowIn ||
			MWM.get_game_state().refstate == freePlay;
	}



	void BPreOwnThrowIn::gainControl(const Time& time) throw() {
		executionFinished = false;
		setExecutionPhase(0);
	}



	DriveVector BPreOwnThrowIn::getCmdBefore(const Time& time) throw() {
		const FieldGeometry& field = MWM.get_field_geometry();

		SPhysGotoPosAvoidObstacles goTo;
		
		vBallPosMem = MWM.get_ball_location(time).pos.toVec();
		vRobotPosMem = MWM.get_robot_location(time).pos;

		Vec& vBallPos = vBallPosMem;
		Vec& vRobotPos = vRobotPosMem;

		Vec vTargetAbsolute = vRobotPos;
		Vec vTargetRelative = vRobotPos - vBallPos;
		Vec vLookingDir;

		// define positions for a throw-in from the right
		switch(WBOARD->getStandardSituationRole()) {
			// 1st ROBOT, alone
			case WhiteBoard::STANDARD_ROLE_ABCDE:
				vTargetRelative.x = 500;
				vTargetRelative.y = -200;
				break;

			// 1st ROBOT
			case WhiteBoard::STANDARD_ROLE_A:
			case WhiteBoard::STANDARD_ROLE_AB:
				vTargetRelative.x = -600;
				vTargetRelative.y = -3000;
				break;

			// 2nd ROBOT
			case WhiteBoard::STANDARD_ROLE_C:
			case WhiteBoard::STANDARD_ROLE_CDE:
				vTargetRelative.x = 800;
				vTargetRelative.y = 400;
				vLookingDir.x = -0.2;
				vLookingDir.y = 1.0;
				break;

			// 3rd ROBOT, safety
			case WhiteBoard::STANDARD_ROLE_E:
			case WhiteBoard::STANDARD_ROLE_DE:
				vTargetAbsolute.x = 0;
				vTargetAbsolute.y = -field.field_length / 2 + 2500;
				break;

			// 4th ROBOT
			case WhiteBoard::STANDARD_ROLE_B:
				vTargetAbsolute.x = -field.field_width / 2 + 1000;
				vTargetAbsolute.y = vBallPos.y;
				break;
				
			// 5th ROBOT
			case WhiteBoard::STANDARD_ROLE_D:
				if(getArea().isFront()) {
					vTargetAbsolute.x = 0;
					vTargetAbsolute.y = field.field_length / 2 - 2500;
				} else {
					vTargetRelative.x = -2000;
					vTargetRelative.y = 200;
				}
				break;

		}

		// mirror positions if it's a throw-in from the left
		if(getArea().isLeft()) {
			vTargetRelative.x *= -1;
			if((vTargetAbsolute - vRobotPos).length() > 50)
				// only if robotpos has been changed
				vTargetAbsolute.x *= -1;
			vLookingDir.x *= -1;
		}

		vTargetAbsolute = vBallPos + vTargetRelative + vTargetAbsolute - vRobotPos;

		if(vLookingDir.length() < 0.1 || (vRobotPos - vTargetAbsolute).length() > 500)
			vLookingDir = vBallPos - vRobotPos;

		goTo.init(vTargetAbsolute, vLookingDir, true);
		return goTo.getCmd(time);
	}



	DriveVector BPreOwnThrowIn::getCmdAfter(const Time& time) throw() {
		SPhysGotoPos goTo;
		SApproachMovingBall approachBall;

		const FieldGeometry& field = MWM.get_field_geometry();

		Vec vBallPos = MWM.get_ball_location(time).pos.toVec();
		Vec vRobotPos = MWM.get_robot_location(time).pos;

		Vec vLookingDir = vBallPos - vRobotPos;
		Vec vTargetAbsolute = vRobotPos;

		switch(WBOARD->getStandardSituationRole()) {
			// 1st ROBOT, tries to get the ball
			case WhiteBoard::STANDARD_ROLE_A:
			case WhiteBoard::STANDARD_ROLE_AB:
				if(getExecutionTime(time) > 500 || getExecutionPhase() > 0) {
					// commitment condition
					if(WBOARD->doPossessBall(time)) {
						std::cout << "(1st Robot) GIVE UP CONTROL!" << std::endl;
						executionFinished = true;
					}

					double ballDistance = (vRobotPos - vBallPos).length();
					double r = max((5000.0 - ballDistance)/5000.0, 0.0);
					
					vLookingDir = r * vLookingDir.normalize() + (1.0 - r) * Vec(0, 1);

					approachBall.setParameters(vLookingDir, 2.0);
					return approachBall.getCmd(time);
				} else {
					// wait
					return DriveVector();
				}

			// 2nd ROBOT, touches the ball
			case WhiteBoard::STANDARD_ROLE_C:
			case WhiteBoard::STANDARD_ROLE_CDE: 
				if(getExecutionPhase() == 2) {
					vTargetAbsolute = vRobotPosMem + Vec(0, 1000);
					vLookingDir = vBallPos - vRobotPos;

					// commitment condition
					if((vRobotPos - vTargetAbsolute).length() < 250) {
						executionFinished = true;
						std::cout << "(2st Robot) GIVE UP CONTROL!" << std::endl;
					}
				} else {
					vTargetAbsolute = vRobotPosMem + 1.2 * (vBallPosMem - vRobotPosMem);
					vLookingDir.x = getArea().isRight() ? -0.2 : 0.2;
					vLookingDir.y = 1.0;
						
					double targetDistance = (vRobotPos - vTargetAbsolute).length();
					double ballPosDistance = (vRobotPos - vBallPosMem).length();
					
					if(getExecutionPhase() < 1 && ballPosDistance < 700)
						setExecutionPhase(1);
					if(targetDistance < ballPosDistance)
						setExecutionPhase(2);
				}

				goTo.init(vTargetAbsolute, vLookingDir, 2.0, false, false);
				return goTo.getCmd(time);
				
			// 5th ROBOT, moves towards the opponents goal
			case WhiteBoard::STANDARD_ROLE_D:
				vTargetAbsolute.x = vRobotPos.x;
				vTargetAbsolute.y = field.field_length / 2 - 2500;
				vLookingDir = vBallPos - vRobotPos;
				
				// commitment condition
				if(getExecutionTime(time) > 1000)
					executionFinished = true;

				goTo.init(vTargetAbsolute, vLookingDir, false);
				return goTo.getCmd(time);
		}

		// if no behavior is defined, give up control
		executionFinished = true;

		return DriveVector();
	}
}
