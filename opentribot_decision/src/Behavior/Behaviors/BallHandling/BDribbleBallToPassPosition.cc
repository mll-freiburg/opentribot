#include "BDribbleBallToPassPosition.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"

#include <cmath>
#include <stdlib.h>
#define DEBUG_PASS 1

namespace Tribots
{

BDribbleBallToPassPosition::
BDribbleBallToPassPosition(double transVel, const Quadrangle &decisionArea,
			   double passProbability)
  : Behavior("BDribbleBallToPassPosition"), 
    transVel(transVel), decisionArea(decisionArea), 
    passProbability(passProbability),
    skill(new SDribbleBallToPos())
    {
        m_bDribblingToSecondPoint =false;
        m_bMessageSend = false;
        m_bDecisionMade = false;
        m_bDoPass = false;
        m_iLostBallLoops = 0;
    }

BDribbleBallToPassPosition::
~BDribbleBallToPassPosition() throw ()
{
  delete skill;
}

void 
BDribbleBallToPassPosition::cycleCallBack (const Time& t) throw()
{
  if (! WBOARD->doPossessBall(t)) {
    if (m_iLostBallLoops>10) {
      m_bDecisionMade = false;
      m_bDoPass = false;
    }
    m_iLostBallLoops++;
  }
  
 //LOUT<<"lbl: "<<m_iLostBallLoops<<"\n";
}

bool 
BDribbleBallToPassPosition::
checkCommitmentCondition(const Time& t) throw()
{
  if ( (!WBOARD->doPossessBall(t)&&m_iLostBallLoops>10) || WBOARD->onlyOneRobot() ||
       WBOARD->onlyTwoRobots()) {
    return false;
  }
  if (WBOARD->doPossessBall(t)) m_iLostBallLoops=0;
  
  if (!m_bDecisionMade) {
    if (DEBUG_PASS) {
      LOUT<<"\% dark_green thin dotted line "<<decisionArea.p1.x<<" "<<decisionArea.p1.y<<" "
          <<decisionArea.p2.x<<" "<<decisionArea.p2.y<<" "
          <<decisionArea.p3.x<<" "<<decisionArea.p3.y<<" "
          <<decisionArea.p4.x<<" "<<decisionArea.p4.y<<" "
          <<decisionArea.p1.x<<" "<<decisionArea.p1.y<<"\n";
    }
      
    Vec _robotPosition = 
      MWM.get_robot_location(t).pos;
    
    // init randomizer (ggf. irgendwo global einbauen)
    //time_t t;
    //time(&t);
    //srand((unsigned int)t);
      
    int _rvalue =rand() % 100;
    if (DEBUG_PASS) LOUT<<"random: "<<_rvalue<<"\n";
    if ( (_rvalue<=100.*passProbability)&&(decisionArea.is_inside(_robotPosition)) )m_bDoPass=true;
      
     m_bDecisionMade=true;
  }
 
  
  if (DEBUG_PASS) LOUT<<"doPass? "<<m_bDoPass<<"\n";
  
  if (!m_bDoPass)
    return false;
    
  return true;    
}

bool 
BDribbleBallToPassPosition::
checkInvocationCondition(const Time& t) throw()
{
  return BDribbleBallToPassPosition::checkCommitmentCondition(t);
}

DriveVector 
BDribbleBallToPassPosition::getCmd(const Time& t) 
  throw(TribotsException) 
{
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  Vec oppGoalPos(0., fgeom.field_length / 2.);  
  
  // \TODO : gets the "unfiltered" robot location 
  Time now;
  RobotLocation robot = MWM.get_slfilter_robot_location(now);
    
  Vec targetPos;
  Vec _vecRightPassPoint(oppGoalPos+Vec(3000,-3000));
  Vec _vecLeftPassPoint(oppGoalPos+Vec(-3000,-3000));
 bool forceHardTurn = 0;
 const BallLocation& ball_exec(MWM.get_ball_location(t));
    if ( robot.pos.x > 0)
    {    // true=right=1, false=left=0
        if( ((robot.pos-_vecRightPassPoint).length()>1000) &&  !m_bDribblingToSecondPoint)
        {
            targetPos = _vecRightPassPoint;
        }
        else
        {
            m_bDribblingToSecondPoint = true;
            targetPos = _vecLeftPassPoint;
        }
    }
    else
    {
        if( ((robot.pos-_vecLeftPassPoint).length()>1000) && !m_bDribblingToSecondPoint)
        {
            targetPos = _vecLeftPassPoint;
        }
        else
        {
            m_bDribblingToSecondPoint = true;
            targetPos = _vecRightPassPoint;
        }
    }
  
    skill->setParameters(targetPos, transVel, forceHardTurn);
  DriveVector dv =skill->getCmd(t);

  Frame2d world2robot =  WhiteBoard::getTheWhiteBoard()->getAbs2RelFrame(t);
  
  if(m_bDribblingToSecondPoint && (fabs(ball_exec.pos.toVec().x)<2000) )
  {
    if(!m_bMessageSend)
    {
      MWM.get_message_board().publish("pass:");
      m_bMessageSend=true;
    dv.kick=1;
    dv.klength=20;
    }
  }
    return dv;
}
  
  
void 
BDribbleBallToPassPosition::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}

void 
BDribbleBallToPassPosition::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
  m_bDribblingToSecondPoint=false;
  m_bMessageSend = false;
//  MWM.get_message_board().publish("cancel_pass:");
}
    

};
