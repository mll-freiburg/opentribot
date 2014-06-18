
#ifndef Tribots_TribotsUDPCommunication_h
#define Tribots_TribotsUDPCommunication_h

#include "NonspecificTaggedUDPCommunication.h"
#include "../Structures/RobotLocation.h"
#include "../Structures/TeammateLocation.h"
#include "../Structures/BallLocation.h"
#include "../Structures/ObstacleLocation.h"
#include "../Structures/VisibleObject.h"
#include "../Structures/GameState.h"
#include "../Structures/FieldGeometry.h"
#include "../Structures/RobotData.h"
#include "../Structures/TacticsBoard.h"
#include "../Structures/DriveVector.h"

#include <string>
#include <vector>


namespace Tribots {

  /** UDP-Verbindung, abgestimmt auf die Datenstrukturen die zwischen
      teamcontrol und robotcontrol zu verschicken sind */
  class TribotsUDPCommunication : public NonspecificTaggedUDPCommunication {
  public:
    /** Konstruktor */
    TribotsUDPCommunication () throw (std::bad_alloc);
    ~TribotsUDPCommunication () throw ();
    /** 'Bye' senden und Verbindung schliessen */
    void close () throw ();

    // Methoden zum lesen/schreiben/anfordern/Anforderung abfragen bestimmter Informationen
    // Return: true, wenn Information geschrieben werden konnte/empfangen wurde
    virtual bool putPing () throw (std::bad_alloc);
    virtual bool getPing () throw ();
    virtual bool putPingRequest () throw (std::bad_alloc);
    virtual bool getPingRequest () throw ();

    virtual bool putBye () throw (std::bad_alloc);
    virtual bool getBye () throw ();

    virtual bool putExitRequest () throw (std::bad_alloc);
    virtual bool getExitRequest () throw ();

    virtual bool putRobotLocation (const RobotLocation&) throw (std::bad_alloc);
    virtual bool getRobotLocation (RobotLocation&) throw ();
    virtual bool putRobotLocation (const RobotLocation&, const TeammateOccupancyGrid&) throw (std::bad_alloc);
    virtual bool getRobotLocation (RobotLocation&, TeammateOccupancyGrid&) throw ();

    virtual bool putTeammateLocation (const std::vector<TeammateLocation>&, unsigned int) throw (std::bad_alloc);  ///< Arg2 ist der ungefaehre Kommunikationsdelay in ms
    virtual bool getTeammateLocation (std::vector<TeammateLocation>&) throw (std::bad_alloc);

    virtual bool putRobotID (unsigned int) throw (std::bad_alloc);
    virtual bool getRobotID (unsigned int&) throw ();

    virtual bool putBallLocation (const BallLocation&) throw (std::bad_alloc);
    virtual bool getBallLocation (BallLocation&) throw ();

    virtual bool putObstacleLocation (const ObstacleLocation&) throw (std::bad_alloc);
    virtual bool getObstacleLocation (ObstacleLocation&) throw (std::bad_alloc);
    virtual bool putObstacleLocationRequest () throw (std::bad_alloc);
    virtual bool getObstacleLocationRequest () throw ();

    virtual bool putVisibleObjectList (const std::vector<VisibleObjectList>&) throw (std::bad_alloc);
    virtual bool getVisibleObjectList (std::vector<VisibleObjectList>&) throw (std::bad_alloc);
    virtual bool putVisibleObjectListRequest () throw (std::bad_alloc);
    virtual bool getVisibleObjectListRequest () throw ();

    virtual bool putRobotData (const RobotData&) throw (std::bad_alloc);
    virtual bool getRobotData (RobotData&) throw (std::bad_alloc);
    virtual bool putRobotDataRequest () throw (std::bad_alloc);
    virtual bool getRobotDataRequest () throw ();

    virtual bool putGameState (const GameState&) throw (std::bad_alloc);
    virtual bool getGameState (GameState&) throw ();

    virtual bool putInGame (const bool&) throw (std::bad_alloc);
    virtual bool getInGame (bool&) throw ();

    virtual bool putRefereeState (const RefereeState&) throw (std::bad_alloc);
    virtual bool getRefereeState (RefereeState&) throw ();

    virtual bool putOwnHalf (const int&) throw (std::bad_alloc);
    virtual bool getOwnHalf (int&) throw ();

    virtual bool putPlayerType (const std::string&) throw (std::bad_alloc);
    virtual bool getPlayerType (std::string&) throw (std::bad_alloc);

    virtual bool putPlayerRole (const std::string&) throw (std::bad_alloc);
    virtual bool getPlayerRole (std::string&) throw (std::bad_alloc);

    virtual bool putPlayerTypeList (const std::vector<std::string>&) throw (std::bad_alloc);
    virtual bool getPlayerTypeList (std::vector<std::string>&) throw (std::bad_alloc);
    virtual bool putPlayerTypeListRequest () throw (std::bad_alloc);
    virtual bool getPlayerTypeListRequest () throw ();

    virtual bool putPlayerRoleList (const std::vector<std::string>&) throw (std::bad_alloc);
    virtual bool getPlayerRoleList (std::vector<std::string>&) throw (std::bad_alloc);
    virtual bool putPlayerRoleListRequest () throw (std::bad_alloc);
    virtual bool getPlayerRoleListRequest () throw ();

    virtual bool putFieldGeometry (const FieldGeometry&) throw (std::bad_alloc);
    virtual bool getFieldGeometry (FieldGeometry&) throw ();
    virtual bool putFieldGeometryRequest () throw (std::bad_alloc);
    virtual bool getFieldGeometryRequest () throw ();

    virtual bool putSLHint (const Vec&, const Angle&) throw (std::bad_alloc);
    virtual bool getSLHint (Vec&, Angle&) throw ();

    virtual bool putMessageBoard (const std::vector<std::string>&) throw (std::bad_alloc);
    virtual bool getMessageBoard (std::vector<std::string>&) throw (std::bad_alloc);
    virtual bool putMessageBoardRequest () throw (std::bad_alloc);
    virtual bool getMessageBoardRequest () throw ();

    virtual bool putVcc (const float&) throw (std::bad_alloc);
    virtual bool getVcc (float&) throw ();

    virtual bool putSynchronisationSignal (const unsigned short int&) throw (std::bad_alloc);
    virtual bool getSynchronisationSignal (unsigned short int&) throw ();

    virtual bool putTacticsBoard (const TacticsBoard&) throw (std::bad_alloc);
    virtual bool getTacticsBoard (TacticsBoard&) throw (std::bad_alloc);

    virtual bool putTacticsBoardForce (const TacticsBoard&) throw (std::bad_alloc);  // auch Taktikboard, aber benutzt um anzuzeigen, dass das Teamcontrol diese Werte annehmen muss
    virtual bool getTacticsBoardForce (TacticsBoard&) throw (std::bad_alloc);

    virtual bool putStandardRequest () throw (std::bad_alloc);  // anfragen nach: RobotLocation, BallLocation, ownHalf, inGame, RefereeState, PlayerType, PlayerRole, Vcc
    virtual bool getStandardRequest () throw ();

    virtual bool putDebugImageRequest () throw (std::bad_alloc);
    virtual bool getDebugImageRequest () throw ();

    virtual bool putScore (unsigned int, unsigned int, unsigned int) throw (std::bad_alloc);  // Spielstand eigeneTore, gegnerischeTore, Anzahl gelbe Karten (gelbe Karten noch nicht implementiert)
    virtual bool getScore (unsigned int&, unsigned int&, unsigned int&) throw ();

    virtual bool putDriveVector (const DriveVector&) throw (std::bad_alloc);
    virtual bool getDriveVector (DriveVector&) throw ();

    virtual bool putSLMirrorHint (const Vec&) throw (std::bad_alloc);
    virtual bool getSLMirrorHint (Vec&) throw ();

  };

}

#endif
