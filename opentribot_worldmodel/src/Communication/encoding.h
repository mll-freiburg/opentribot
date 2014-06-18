
#ifndef _Tribots_encoding_h_
#define _Tribots_encoding_h_

#include "../Structures/RobotLocation.h"
#include "../Structures/TeammateLocation.h"
#include "../Structures/BallLocation.h"
#include "../Structures/ObstacleLocation.h"
#include "../Structures/VisibleObject.h"
#include "../Structures/RobotData.h"
#include "../Structures/GameState.h"
#include "../Structures/FieldGeometry.h"
#include "../Structures/TacticsBoard.h"
#include <vector>
#include <string>

namespace Tribots {

  // Funktionen, um Strukturen fuer die Kommunikation geeignet zu codieren.
  // bufferSizeXXX (x) liefert die Puffergroesse zurueck, die benoetigt wird, um x zu codieren
  // encodeXXX (buffer, x) schreibt x nach buffer. buffer muss vorher angelegt und ausreichend gross sein. Liefert die benoetigte Puffergroesse
  // decodeXXX (buffer, bufferlen, x) liest x aus buffer aus und liefert true bei Erfolg. bufferlen ist die Groesse von buffer

  unsigned int bufferSizeRobotLocation (const RobotLocation&, const TeammateOccupancyGrid&) throw ();
  unsigned int encodeRobotLocation (char*, const RobotLocation&, const TeammateOccupancyGrid&) throw ();
  bool decodeRobotLocation (const char*, unsigned int, RobotLocation&, TeammateOccupancyGrid&) throw ();

  unsigned int bufferSizeBallLocation (const BallLocation&) throw ();
  unsigned int encodeBallLocation (char*, const BallLocation&) throw ();
  bool decodeBallLocation (const char*, unsigned int, BallLocation&) throw ();

  unsigned int bufferSizeObstacleLocation (const ObstacleLocation&) throw ();
  unsigned int encodeObstacleLocation (char*, const ObstacleLocation&) throw ();
  bool decodeObstacleLocation (const char*, unsigned int, ObstacleLocation&) throw ();

  unsigned int bufferSizeVisibleObjectList (const std::vector<VisibleObjectList>&) throw ();
  unsigned int encodeVisibleObjectList (char*, const std::vector<VisibleObjectList>&) throw ();
  bool decodeVisibleObjectList (const char*, unsigned int, std::vector<VisibleObjectList>&) throw ();

  unsigned int bufferSizeRobotData (const RobotData&) throw ();
  unsigned int encodeRobotData (char*, const RobotData&) throw ();
  bool decodeRobotData (const char*, unsigned int, RobotData&) throw ();

  unsigned int bufferSizeGameState (const GameState&) throw ();
  unsigned int encodeGameState (char*, const GameState&) throw ();
  bool decodeGameState (const char*, unsigned int, GameState&) throw ();

  unsigned int bufferSizeOwnHalf (const int&) throw ();
  unsigned int encodeOwnHalf (char*, const int&) throw ();
  bool decodeOwnHalf (const char*, unsigned int, int&) throw ();

  unsigned int bufferSizeStringList (const std::vector<std::string>&) throw ();
  unsigned int encodeStringList (char*, const std::vector<std::string>&) throw ();
  bool decodeStringList (const char*, unsigned int, std::vector<std::string>&) throw ();

  unsigned int bufferSizeFieldGeometry (const FieldGeometry&) throw ();
  unsigned int encodeFieldGeometry (char*, const FieldGeometry&) throw ();
  bool decodeFieldGeometry (const char*, unsigned int, FieldGeometry&) throw ();

  unsigned int bufferSizeSLHint (const Vec&, const Angle&) throw ();
  unsigned int encodeSLHint (char*, const Vec&, const Angle&) throw ();
  bool decodeSLHint (const char*, unsigned int, Vec&, Angle&) throw ();

  unsigned int bufferSizeVcc (const float&) throw ();
  unsigned int encodeVcc (char*, const float&) throw ();
  bool decodeVcc (const char*, unsigned int, float&) throw ();

  unsigned int bufferSizeTacticsBoard (const TacticsBoard&) throw ();
  unsigned int encodeTacticsBoard (char*, const TacticsBoard&) throw ();
  bool decodeTacticsBoard (const char*, unsigned int, TacticsBoard&) throw ();

  unsigned int bufferSizeScore (unsigned int, unsigned int, unsigned int) throw (); // own score, opponent score, yellow cards
  unsigned int encodeScore (char*, unsigned int, unsigned int, unsigned int) throw ();
  bool decodeScore (const char*, unsigned int, unsigned int&, unsigned int&, unsigned int&) throw ();

}


#endif
