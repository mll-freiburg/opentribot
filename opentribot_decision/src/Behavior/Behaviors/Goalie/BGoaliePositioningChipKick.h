
#ifndef Tribots_BGoaliePositioningChipKick_h
#define Tribots_BGoaliePositioningChipKick_h

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots {

  /** Positioniert den Goalie in Abhaengigkeit der Ballposition und -geschwindigkeit 
      auf der eigenen Torlinie, um hohe Schuesse abwehren zu koennen und bewegt sich
      ggf. auf der Grundlinie */
  class BGoaliePositioningChipKick : public Behavior {
  public:
    /** Konstruktor, arg1: Anfahrskill */
    BGoaliePositioningChipKick (SPhysGotoPos* =NULL) throw ();
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();  ///< Ball vor max. 1.8 s in opponentHalf gesehen? 
    bool checkCommitmentCondition (const Time&) throw ();  ///< dito
    bool checkCondition (const Time&) throw ();
    void gainControl (const Time&) throw (TribotsException);
    void cycleCallBack (const Time&) throw ();
    /** liest den Parameter 'GoalieChipKickPositioning', der festlegt, ab 
        welcher Ball-y-Position das Behavior aktiv wird. Moegliche Werte sind:
         - 'nie'
         - 'immer'
         - 'GegnerHaelfte' (y=0), default
         - 'GegnerDrittel' (y=field_length/6)
         - 'GegnerZweiDrittel' (y=-field_length/6)
         - 'GegnerViertel' (y=field_length/4);
         - 'GegnerDreiViertel' (y=-field_length/4);
         - y-Wert */
    void updateTactics (const TacticsBoard&) throw ();

  private:
    Vec home;  ///< die Warteposition des Goalie
    double maxextend;  ///< die maximale Auslenkung der Zielposition auf der Grundlinie

    XYRectangle opponentHalf;  ///< der Bereich des Feldes, aus dem hohe Fernschuesse kommen koennen (muss nicht mit gegnerischen Haelfte identisch sein)
    XYRectangle ownHalf;  ///< Feld ohne opponentHalf
    XYRectangle ownHalfFront;   ///< Uebergangsbereich von opponentHalf in ownHalf
    XYRectangle workingArea;  ///< der Bereich, in dem der Torwart sich mit diesem Behavior bewegen soll (nicht alles, um keine Hindernisse umzurempeln)
    XYRectangle toleranceArea1;  ///< die ersten 80cm der opponentHalf
    XYRectangle toleranceArea2;  ///< weitere 40cm der opponentHalf
    bool wasInToleranceArea2;
    bool wasInToleranceArea2TwoTimes;

    Vec latestBallPositionInOpponentHalf;  ///< letzte Ballposition innerhalb von opoonentHalf
    Vec latestBallPositionInOwnHalf;  ///< letzte Ballposition innerhalb von ownHalf
    bool ballSeenInOwnHalf;  ///< wurde der Ball bereits in ownHalf gesehen?
    Time ballSeenInOpponentHalf;  ///< der Zeitpunkt, zu dem letztmalig der Ball in opponentHalf gesehen wurde
    unsigned int numBallSeenInOpponentHalf;   ///< Anzahl Zyklen, in denen der Ball in der gegnerischen Haelfte gesehen wurde seit dem letzten Mal, als er in der eigenen Haelfte gesehen wurde
    Angle tghead;  ///< die Zielausrichtung (zum Ball, wenn der Ball in der gegnerischen Haelfte liegt, ansonsten bisherige Ausrichtung)
    unsigned int numInOwnHalf;
    unsigned int numInOwnHalfFront;

    SPhysGotoPos own_goto_pos_skill;
    SPhysGotoPos* goto_pos_skill;
    
    void setOpponentHalf (double y) throw ();  ///< setzt ownHalf und opponentHalf, wobei opponentHalf.y > y gelten soll 
  };

}

#endif
