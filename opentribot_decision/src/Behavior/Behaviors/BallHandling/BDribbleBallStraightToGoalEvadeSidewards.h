#ifndef _BDRIBBLEBALLSTRAIGHTTOGOALEVADESIDEWARDS_H_
#define _BDRIBBLEBALLSTRAIGHTTOGOALEVADESIDEWARDS_H_

#include "../../Behavior.h"
#include "../../Skills/BallHandling/SDribbleBallStraightToPosEvadeSidewards.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{

/** 
 * Dribbelt den Ball auf der geraden Verbindung Richtung Tor und weicht 
 * Hindernissen dabei durch Seitwärtsbewegungen aus. Funktioniert nur, solange
 * das Tor innerhalb eines +/-45° Kegels vor dem Roboter (bezüglich der Achse 
 * Robotermittelpunkt - Kicker) ist.
 */
class BDribbleBallStraightToGoalEvadeSidewards : public Tribots::Behavior
{
public:
  /**
   * Erzeugt eine Instanz dieses Verhaltens, wobei die translatorische 
   * Geschwindigkeit begrenzt werden kann. Außerdem kan ein Aktivierungsbereich
   * angegeben werden. Hier ist zu beachten, dass dieses Verhalten gerade aus
   * den Seitenbereichen des Tores nicht sonderlich effektiv ist und diese 
   * daher ausgeschlossen werden sollten.
   * 
   * \param transVel maximale translatorische Geschwindigkeit
   * \param activationArea Bereich, in der diese Anfahrt aktiv werden darf.
   *                       Wird kein Bereich übergeben, wird das Behavior auf
   *                       dem ganzen Feld abgesehen von 1000mm breiten
   *                       Streifen an den Seitenlinien und einem 1000 mm 
   *                       breiten Streifen an der gegnerischen Grundlinie
   *                       aktiv.
   */
	BDribbleBallStraightToGoalEvadeSidewards(double transVel = 2.3,
                                           const Area* activationArea = 0);
	virtual ~BDribbleBallStraightToGoalEvadeSidewards() throw();
  
  /** Tor befindet sich innerhalb eines +/- 45° Winkels vor dem Roboter und
   *  Roboter hat den Ball */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Tor befindet sich innerhalb eines +/- 25° Winkels vor dem Roboter und 
   *  Roboter hat den Ball und Ball ist in activationArea */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);  
	
	virtual void updateTactics (const TacticsBoard& tb) throw ();
  
protected:
  bool doDribbleToMiddleFirst;
  double transVel;
  Area* activationArea;
  SDribbleBallStraightToPosEvadeSidewards* skill;
  int corner;
  bool changed;
  int presentPointToPos;
	
  bool    considerObstaclesInGoal;
  double  obstacles_behind_goal_distance;
};

};

#endif //_BDRIBBLEBALLSTRAIGHTTOGOALEVADESIDEWARDS_H_
