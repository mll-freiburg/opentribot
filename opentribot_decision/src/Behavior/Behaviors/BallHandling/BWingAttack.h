#ifndef _Tribots_BWingAttack_H_
#define _Tribots_BWingAttack_H_

#include "../../Behavior.h"
#include "../../Skills/BasicMovements/SBoostToPos.h" 

namespace Tribots
{
  
class SDribbleBallToPosInterface;
class SDribbleBallStraightToPosEvadeSidewards;

/** 
  * Dribbelt den Ball entlang der Seitenlinie nach vorne bis in den Eigenmove
  * Bereich. Geht in der N�he der Seitenlinien in der eigenen H�lfte an.
  * Optional kann eingestellt werden, dass ein einmal eingeleiteter 
  * Fl�gelangriff vorzeitig abgebrochen wird, wenn der Weg zum Tor v�llig frei
  * ist. Das Verhalten wird sofort abgebrochen, wenn sich vor dem Roboter ein
  * Hindernis auftut; es hat keine eigene Hindernisausweichmethode.
  * 
  * �ber das Taktikboard kann die Aktivierungswahrscheinlichkeit 
  * (Fluegelangriff=[immer oft manchmal selten nie]) und die Erlaubnis zum
  * fr�hzeitigen Abbruch (FluegelangriffAusstieg=[ja nein]) gesetzt werden.
  *
  * Dieses Verhalten ersetzt die F�higkeit von BDribbleToPassPosition, das
  * bisher f�r Angriffe �ber die Fl�gel zust�ndig war. Ein Angriff �ber die
  * Seiten hat sich gegen manche Teams, die zwischen Ball und Tor verteidigen
  * und deshalb immer mehr zur Mitte des Spielfeldes stehen, als vorteilhaft
  * erwiesen.
  * 
  * \attention Die Abstimmung mit BEigenMove ist entscheidend. Der Zielpunkt
  *            dieses Verhaltens muss im Aktivierungsbereich des Eigenmoves
  *            liegen.
 */
class BWingAttack : public Tribots::Behavior
{
public:
  /** Konstruktor des Fl�gelangriffs. Erwartet eine Zielgeschwindigkeit, eine
    * Aktivierungswahrscheinlichkeit und ein boolean, dass angibt, ob der
    * Fl�gelangriff fr�hzeitig abgebrochen werden darf.
    *
    * \param transVel die Zielgeschwindigkeit
    * \param activationProbability Aktivierungswahrscheinlichkeit
    * \param earlyExit darf der Fl�gelangriff fr�hzeitig abgebrochen werden?
    */
  BWingAttack(double transVel,
              double activationProbability,
              bool earlyExit);
	virtual ~BWingAttack() throw();
  
  /** Bleibt an, solange der Roboter den Ball hat, kein Hindernis vor ihm ist,
    * der Weg zum Tor nicht v�llig frei ist (wenn fr�hzeitiger Abbruch erlaubt),
    * der Roboter grob in die richtige Richtung schaut (nach vorne) und der 
    * Zielpunkt noch nicht erreicht wurde. */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** Geht in der eigenen H�lfte im Bereich der Seitenlinien bei Ballbesitz an,
    * wenn der Roboter grob in die richtige Richtung schaut, vor ihm und zur
    * Seite platz ist und das Zufallsexperiment gegen die 
    * Aktivierungswahrscheinlichkeit positiv ausgeht */  
  virtual bool checkInvocationCondition(const Time&) throw();
  
  void cycleCallBack (const Time& t) throw();
  
  /** F�hrt erst richtung der Seitenlinie, bis maximal 1,5m entfernt, f�hrt
    * dann geradewegs den Zielpunkt im Bereich der Eigenmoveaktivierungsarea
    * an. */
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  
  virtual void gainControl(const Time&) throw(TribotsException);
  virtual void loseControl(const Time&) throw(TribotsException);  
  
  /** Liest die Werte der beiden Parameter 
    * Fluegelangriff=[immer oft manchmal selten nie] und  
    * FluegelangriffAusstieg=[ja nein] aus dem Taktikboard. und
    * boosting=[ja nein]
    */
  virtual void updateTactics (const TacticsBoard&) throw ();
  
protected:
  bool m_bDribblingToSecondPoint; ///< erster Zwischenpunkt erreicht
  bool m_bDecisionMade; ///< schon gegen die Aktivierungswahrscheinlichkeit gew�rfelt?
  bool mayBecomeActive; ///< darf in dieser Ballbesitzphase aktiviert werden?
  int  m_iLostBallLoops;///< wie lange ist der Ball bereits verloren?
  bool reached; ///< Zielpunkt erreicht? Dann im n�chsten Zyklus abbrechen
      
  double transVel; ///< Zielgeschwindigkeit
  double activationProbability; ///< Aktivierungswahrscheinlichkeit
  bool earlyExit; ///< darf fr�hzeitig abbrechen?
  bool boosting;  ///< bosting erlaubt?
  bool useEvade;  ///< Zielstrebiges Dribbeln verwenden?
  SDribbleBallToPosInterface* skill;      ///< skill, der zum Dribbeln verwendet wird
  SDribbleBallStraightToPosEvadeSidewards* skillEvade;
  SBoostToPos*       fast_skill; ///< skill zum schnellen "dribbeln", keine Feinregelung
  
  Vec targetPoint; ///< Punkt, der im Vorfeld angefahren werden soll
  Skill* activeSkill;

};

};

#endif //_Tribots_BDribbleBallToPassPosition_H_
