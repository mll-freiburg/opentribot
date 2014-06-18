#ifndef _WHITEBOARD_H_
#define _WHITEBOARD_H_

#include "../Fundamental/Time.h"
#include "../Fundamental/Frame2D.h"
#include "../Fundamental/geometry.h"
#include "../Fundamental/ConfigReader.h"
#include <string>
#include <sstream>

namespace Tribots {

  /** Singleton, das Standardabfragen und Prädikate bereitstellt. 
   *  Rechenzeitaufwändige Anfragen sollen ihre Ergebnisse zwischenspeichern
   *  und nur neu berechnen, wenn Zykluszähler oder angefragter Zeitstempel
   *  abweichen.
   *
   *  Dieses WhiteBoard ist ersteinmal eine Minimalversion und bietet keinen
   *  einheitlichen Mechanismus um Berechnungsergebnisse zwischenzuspeichern 
   *  und zu verwalten. Könnte man in der Zukunft noch ausbauen...
   */
  class WhiteBoard {
  public:
    /** von dieser Klasse werden spezielle Datentypen abgeleitet, die
     *  verschiedene Werte mit Zeitstempel und Zyklus versehen 
     *  zwischenpeichern können. 
     */
    class Data {
    public:
      Time t;
      unsigned long cycle;

      virtual ~Data() {}
    };

    /** liefert einen Pointer auf die Instanz des WhiteBoards. */
    static WhiteBoard* getTheWhiteBoard();
    
    /** Liefert die Matrix zur Umrechnung von absoluten nach relativen 
     *  Koordinaten. 
     *  \attention Die Zeitstempel von umgerechneten Positionen und angefragter
     *             Umrechnungsmatrix sollten übereinstimmen!
     */
    const Frame2d& getAbs2RelFrame(const Time& t);
    /** Liefert die Matrix zur Umrechnung von relativen nach absoluten 
     *  Koordinaten. 
     *  \attention Die Zeitstempel von umgerechneten Positionen und angefragter
     *             Umrechnungsmatrix sollten übereinstimmen!
     */
    const Frame2d& getRel2AbsFrame(const Time& t);

    /** Test, ob ball im gegnerischen tor, wobei die tiefe des tores mit 2 
     *  multipliziert wird, um fehler zu verhindern */
    bool isBallInOppGoal(const Time& t);
    
    /** Prueft, ob der ball gedribbelt werden kann */
    bool canDribbleBall();
    /** Prüft, ob der Roboter selbst im Ballbesitz ist. */
    bool doPossessBall(const Time& t);
    void resetPossessBall();
    /**Nur true setzen wenn man den keinen Ballbesitz erhalten möchte obwohl man welchen haben könnte, unbedingt immer wieder nach Gebrauch ausschalten*/
    bool m_bDoNotGetBallPossession;
    /** gibt an, ob ein anderer Roboter des Teams im Ballbesitz ist */
    bool teamPossessBall();
    bool teamPossessBallExtended();
    /** gibt an, ob ein anderer Roboter des Teams in der Naehe des Balls ist */
    bool teammateNearBall();    

    /** is true, if there are only two robots in the game */
    bool onlyTwoRobots() const;
    bool onlyOneRobot() const;
    bool onlyThreeRobots() const;
    bool onlyFourRobots() const;

    /** gibt an wieviele Roboter gerae im Spiel sind **/
    
    unsigned int getActiveRobots() const;

    bool detectPassedBall(const Time& texec);
    bool receivePass() const;
    bool receivePlannedPass(unsigned int playerId) const;
    void resetPlannedPass();
    bool touchedBall() const;
    bool doCounterAttack() const;
    bool receiveSetPlayShortPass() const;

    /**
     * Die folgenden Methoden können zur Implementation von Standardsituationen
     * nützlich sein. Es gibt maximal drei Feldspieler denen je eine Rolle
     * A, B oder C zugewiesen wird. Durch einen externen Mechanismus 
     * (derzeit ist der Spielertyp, später die Taktikkommunikation zuständig)
     * wird sicher gestellt, dass jeder Spiele eine andere Rolle erhält. Das
     * kann von einem Verhalten dazu verwendet werden, Spielfeldpositionen
     * an die drei Spieler zu verteilen. Diese Rollenvergabe ist unabhängig
     * vom PlayerType und hat nichts mit den vom Spielertyp bereitgestellten
     * Rollen zu tun; die "Standardsituationsrollen" sind für alle Spielertypen
     * gleich.
     *
     * Bei 2 Feldspieler erhält einer die Rolle A und der andere die Rolle
     * BC. Bei nur einem Feldspieler, muss dieser alle Rollen "übernehmen" und
     * erhält die Rolle ABC.
     *
     * Die Enkodierung über die Bits darf verwendet werden und darf in Zukunft
     * nicht geändert werden.
     * \attention Bei den Standardrollen für den 4ten und 5ten Spieler gilt das
     * nicht mehr
     */
    enum { STANDARD_ROLE_A=1, STANDARD_ROLE_B=2, STANDARD_ROLE_C=4,
           STANDARD_ROLE_D=8, STANDARD_ROLE_E=16, STANDARD_ROLE_AB=3,
           STANDARD_ROLE_BC=6, STANDARD_ROLE_ABC=7, STANDARD_ROLE_CD=12,
           STANDARD_ROLE_DE=24, STANDARD_ROLE_CE=20, STANDARD_ROLE_CDE=28, 
           STANDARD_ROLE_ABCDE=31};
    
    /** Die aktuelle Rolle in Standardsituationen abfragen. */
    int getStandardSituationRole() const;
    /** 
     * Die aktuelle Rolle bei Standardsituationen setzen. 
     * \attention Diese Methode ist Implementierungen von PlayerType 
     *            vorbehalten, um diesen Wert an Hand der spielerspezifischen
     *            Kommunikation festzulegen. 
     */
    void setStandardSituationRole(int role);
    
    void setZonePressureRole(std::string role);
    std::string getZonePressureRole() const;

    /**
     * prueft das Messageboard auf interessante Inhalte, z.B. fuer 
     * teamPosessesBall 
     */
    void checkMessageBoard();

    /**
     *  Hindernisvermeidung auf der Ebene der Fahrtvektoren.
     *  Berechnet für einen gewünschten Fahrtvektor einen möglichst 
     *  nahegelegenen Ausweichvektor, der zu keinen Kollisionen mit 
     *  Hindernissen führt.
     *  Diese Methode sollte verwendet werden, indem man erst den optimalen
     *  Fahrtvektor für ein Ziel bestimmt, dann mit dieser Methode einen
     *  nahegelgenenen sicheren Ausweichvektor berechnet. Diesen
     *  Ausweichvektor verwendet man dann als vtrans im Drivevektor.
     */
    Vec calculateEvasiveMovement(const Vec& desiredVtrans, 
                                 const Vec& targetPos, const Time& t);

    /**
     * Hindernisvermeidung auf der Ebene des Zielpunktes. Berechnet für einen
     * gegebenen Zielpunkt einen Ausweichzielpunkt, der zuerst angefahren 
     * werden sollte, um eine Kollision zu vermeiden.
     */
    Vec calculateEvasiveTarget(const Vec& desiredTarget)
    { return desiredTarget; }

    /**
     * schaut nach, wo im Tor eine freie Position ist, auf die man zielen 
     * sollte. Voruebergehende Loesung. Hat praktische keine Auswirkungen mehr
     */
    Vec getFreeGoalPos(const Time& tt);
    bool isFreeShootCorridor(const Time& tt);
    bool isFreeDribbleCorridor(Vec targetPos, const Time& tt);
    Vec getEvasiveManeuverTarget(Vec targetPos, const Time& tt);
		
	 /**
	  * modelliert die flugparabel des balles, gibt die hoehe in der sich der ball
	  * in der Distanz (distance_mm) befindet zurueck, wenn mit einer staerke von
	  * klength_ms geschossen wurde
	  **/
	 double getKickTargetHeight(double distance_mm, unsigned char klength_ms);
	 
	 /**
	  * berechnet die Staerke mit der geschossen werden muss um denn ball in einer
	  * Entfernung von distance_mm auf eine hoehe von height_mm zu befoerdern
	  * ueber reachable kann abgefragt werden ob dieses ziel zu errreichen ist
	  **/
	 unsigned char getKickLength(double distance_mm, double height_mm, bool* reachable);

    virtual ~WhiteBoard();

    void readConfigs(const ConfigReader & cfg);

  protected:


    WhiteBoard();
    static WhiteBoard* instance;
    int owns_ball_pixel_distance; //Player Specific Owns Ball (finally av


    Data* ballInOppGoalData;
    Data* possesBallData;
    Data* abs2relData;
    Data* rel2absData;
    Data* shootCorridorData;
    Data* freeGoalPosData;
    Data* dribbleCorridorData;
    Data* passPlayedData;
    

    int frames_ball_owned;
    Time time_ball_owned;
    Time time_ball_lost;
    unsigned int cycles_without_team_posses_ball;
    unsigned int cycles_without_team_possess_ball_extended;
    unsigned int cycles_without_advanced_team_posses_ball;  
    unsigned int cycles_without_active_robots; 
    unsigned int cycles_without_receive_pass;
    unsigned int cycles_without_receive_spsp;
    unsigned int cycles_without_planned_pass;
    unsigned int cycles_without_touched_ball;
    unsigned int cycles_without_nach_vorne;    
    unsigned int number_active_robots;
    
    unsigned int planned_pass_receiver;
    
    // high kicker parabel parameters (see code)
    double ms_x_a;
    double ms_x_b;
    double ms_x_c;
    double ms_y_a;
    double ms_y_b;
    double ms_y_c;

    int standardSituationRole;
    std::string zonePressureRole;
    unsigned long lastMessageBoardCheck;
    
    bool m_bFreeWayToTarget;             // \TODO: aufräumen: macht Unsinn bei
    int m_iEvasiveManeuverMainDirection; // mehrfachen aufrufen in einem Zyklus

    class BoolData : public Data {
    public:
      bool b;
    };

    class Frame2dData : public Data {
    public:
      Frame2d frame;
    };

    class VecData : public Data {
    public:
      Vec v;
    };

    class CorridorData : public Data {
    public:
      Vec pos;
      bool isFree;
      Vec refPos;
    };

    void checkDribbleCorridor(CorridorData* data,
                              Vec p_targetPos, const Time& t);   
  };

}

#define WBOARD Tribots::WhiteBoard::getTheWhiteBoard()
#endif
