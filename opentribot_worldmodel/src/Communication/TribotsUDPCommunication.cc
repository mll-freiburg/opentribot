
#include "TribotsUDPCommunication.h"
#include "encoding.h"
#include <cstring>

using namespace Tribots;
using namespace std;

// Tags sind wiefolgt belegt (gerade Zahl=Wert, ungerade Zahl=Request).
// Prioritaeten fuer Werte in Klammern; Requests haben stets Prioritaet 0
//
// 0/1: Ping (0)
// 2: RobotLocation (0)
// 4: BallLocation (0)
// 6/7: ObstacleLocation (1)
// 8/9: VisibleObjectList (1)
// 10/11: RobotData (1)
// 12: GameState (0)
// 14: InGame (0)
// 16: RefereeState (0)
// 18: OwnHalf (0)
// 20: PlayerType (0)
// 22: PlayerRole (0)
// 24/25: PlayerTypeList (0)
// 26/27: PlayerRoleList (0)
// 28/29: FieldGeometry (0)
// 30: SLHint (0)
// 32/33: MessageBoard (0)
// 34: Vcc (0)
// 37: StandardRequest (0)
// 38: TeammateLocation (0)
// 40: RobotID (0)
// 42: TacticsBoard (0)
// 45: DebugImageRequest (0)
// 46: RobotLocation & TeammateOccupancyGrid (0)
// 48: Spielstand (0)
// 50: DriveVector (0)
// 52: SL-Spiegelungshinweis (0)
// 53/54: RegionList (0)
// 56: TacticsBoardForce (0)
// 250: ExitRequest (0)
// 252: Synchronisationssignal (0)
// 254: Bye (0)


namespace {
  // um diese beiden Variablen nicht in jeder einzelnen Methode erneut deklarieren zu muessen, hier:
  const char* retbuf;
  unsigned int retlen;
  const char magic_bytes [] = { 15, -6 };
  const char exit_code [] = { 93, -18, -109, -54, 70 };
  const unsigned int exit_code_length = 5;
}


TribotsUDPCommunication::TribotsUDPCommunication () throw (std::bad_alloc) : NonspecificTaggedUDPCommunication (magic_bytes) {;}

TribotsUDPCommunication::~TribotsUDPCommunication () throw () { 
  close(); 
}

void TribotsUDPCommunication::close () throw () {
//  clear_send_buffer();
//  putBye ();
//  send ();
  NonspecificTaggedUDPCommunication::close ();
}



bool TribotsUDPCommunication::putPing () throw (std::bad_alloc) {
  return socket.put (0,NULL,0);
}
bool TribotsUDPCommunication::getPing () throw () {
  return socket.get (0,retbuf,retlen);
}
bool TribotsUDPCommunication::putPingRequest () throw (std::bad_alloc) {
  return socket.put (1,NULL,0);
}
bool TribotsUDPCommunication::getPingRequest () throw () {
  return socket.get (1,retbuf,retlen);
}

bool TribotsUDPCommunication::putExitRequest () throw (std::bad_alloc) {
  return socket.put (250,exit_code,exit_code_length);
}
bool TribotsUDPCommunication::getExitRequest () throw () {
  bool okay = (socket.get (250,retbuf,retlen)) && (retlen==exit_code_length);
  if (!okay)
    return false;
  for (unsigned int i=0; i<exit_code_length; i++) {
    okay &= (retbuf[i]==exit_code[i]);
  }
  return okay;
}

bool TribotsUDPCommunication::putBye () throw (std::bad_alloc) {
  return socket.put (254,NULL,0);
}
bool TribotsUDPCommunication::getBye () throw () {
  return socket.get (254,retbuf,retlen);
}
bool TribotsUDPCommunication::putRobotLocation (const RobotLocation& src1, const TeammateOccupancyGrid& src2) throw (std::bad_alloc) {
  char buffer [bufferSizeRobotLocation (src1, src2)];
  unsigned int len=encodeRobotLocation (buffer, src1, src2);
  return socket.put (46,buffer,len);
}

bool TribotsUDPCommunication::getRobotLocation (RobotLocation& dest1, TeammateOccupancyGrid& dest2) throw () {
  return socket.get (46,retbuf,retlen) && decodeRobotLocation (retbuf, retlen, dest1, dest2);
}
bool TribotsUDPCommunication::putRobotLocation (const RobotLocation& src) throw (std::bad_alloc) {
  char buffer [14];
  write_signed_short (buffer, src.pos.x);
  write_signed_short (buffer+2, src.pos.y);
  write_signed_short (buffer+4, 1000*src.heading.get_rad());
  write_signed_short (buffer+6, 1000*src.vtrans.x);
  write_signed_short (buffer+8, 1000*src.vtrans.y);
  write_signed_short (buffer+10, 1000*src.vrot);
  buffer[12]=(src.kick ? 1 : 0);
  buffer[13]=(src.stuck.robot_stuck  ? 1 : 0);
  return socket.put (2,buffer,14);
}
bool TribotsUDPCommunication::getRobotLocation (RobotLocation& dest) throw () {
  if (socket.get (2,retbuf,retlen) && retlen>=14) {
    dest.pos.x = read_signed_short (retbuf);
    dest.pos.y = read_signed_short (retbuf+2);
    dest.heading.set_rad (0.001*read_signed_short (retbuf+4));
    dest.vtrans.x = 0.001*read_signed_short (retbuf+6);
    dest.vtrans.y = 0.001*read_signed_short (retbuf+8);
    dest.vrot = 0.001*read_signed_short (retbuf+10);
    dest.kick = (retbuf[12]==1);
    dest.stuck.robot_stuck = (retbuf[13]==1);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putTeammateLocation (const std::vector<TeammateLocation>& src, unsigned int delay) throw (std::bad_alloc) {
  const unsigned int buflenperrobot = 15;
  const unsigned int bufsize = buflenperrobot*src.size()+1;
  char* buffer = new char [bufsize];
  buffer[0] = static_cast<char>(delay>>4);
  for (unsigned int i=0; i<src.size(); i++) {
    write_signed_short (buffer+1+i*buflenperrobot, src[i].pos.x);
    write_signed_short (buffer+1+i*buflenperrobot+2, src[i].pos.y);
    write_signed_short (buffer+1+i*buflenperrobot+4, 1000*src[i].heading.get_rad());
    write_signed_short (buffer+1+i*buflenperrobot+6, 1000*src[i].vtrans.x);
    write_signed_short (buffer+1+i*buflenperrobot+8, 1000*src[i].vtrans.y);
    write_signed_short (buffer+1+i*buflenperrobot+10, 1000*src[i].vrot);
    unsigned char mb = (src[i].kick ? 1 : 0); // Bit 0, Bit 1 reserviert fuer stark/schwach (zukuenftige Erweiterung)
    mb|=(src[i].stuck.robot_stuck  ? 4 : 0); // Bit 2
    mb|=(src[i].number<<3);  // Bits 3..7, beschraenkt die uebertragbaren Roboter-Nummern auf 0..31
    buffer[1+i*buflenperrobot+12]=static_cast<char>(mb);
    unsigned int allcells=0;
    for (unsigned int j=0; j<16; j++)
      allcells = (allcells<<1)+(src[i].occupancy_grid.cells[j]?1:0);
    buffer[1+i*buflenperrobot+13]=static_cast<char>(static_cast<unsigned char>(allcells));
    buffer[1+i*buflenperrobot+14]=static_cast<char>(static_cast<unsigned char>(allcells>>8));
  }
  bool success = socket.put (38,buffer,bufsize);
  delete [] buffer;
  return success;
}
bool TribotsUDPCommunication::getTeammateLocation (std::vector<TeammateLocation>& dest) throw (std::bad_alloc) {
  // Kompatibilitaet beachten: Uebertragung mit 13 Byte/Roboter ohne OccupancyGrid bzw. 15 Byte/Roboter mit OccupancyGrid
  // Eingebaut 20. Mai 2006, kann nach einiger Zeit zugunsten der 15 Byte-Version vereinfacht werden
  if (socket.get (38,retbuf,retlen) && retlen>0) {
    unsigned int buflenperrobot = 15;
    bool with_occupancy_grid = true;
    unsigned int num = (retlen-1)/buflenperrobot;
    if (num*buflenperrobot+1!=retlen) {
      buflenperrobot = 13;
      with_occupancy_grid = false;
      num = (retlen-1)/buflenperrobot;
      cerr << "ACHTUNG: veraltete Kommunikation => aktualisieren und erneut uebersetzen!\n";
    }
    dest.resize (num);
    int delay = (static_cast<unsigned int>(static_cast<unsigned char>(retbuf[0]))<<4);
    Time timestamp;
    timestamp.add_msec (-delay);
    const char* pt=retbuf+1;
    for (unsigned int i=0; i<num; i++) {
      dest[i].pos.x = read_signed_short (pt);
      dest[i].pos.y = read_signed_short (pt+2);
      dest[i].heading.set_rad (0.001*read_signed_short (pt+4));
      dest[i].vtrans.x = 0.001*read_signed_short (pt+6);
      dest[i].vtrans.y = 0.001*read_signed_short (pt+8);
      dest[i].vrot = 0.001*read_signed_short (pt+10);
      unsigned char mb = static_cast<unsigned char>(pt[12]);
      dest[i].kick = (mb&1);
      dest[i].stuck.robot_stuck = (mb&4);
      dest[i].number = (mb>>3);
      dest[i].timestamp=timestamp;
      if (with_occupancy_grid) {
        unsigned int og = static_cast<unsigned char> (pt[13])+(static_cast<unsigned int>(static_cast<unsigned char> (pt[14]))<<8);
        for (unsigned int j=0; j<16; j++) {
          dest[i].occupancy_grid.cells[j] = (og&1);
          og >>= 1;
        }
      }
      pt+=buflenperrobot;
    }
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putRobotID (unsigned int id) throw (std::bad_alloc) {
  char buffer[1];
  buffer[0] = static_cast<char>(id);
  return socket.put (40, buffer, 1);
}

bool TribotsUDPCommunication::getRobotID (unsigned int& id) throw () {
  if (socket.get (40,retbuf,retlen) && retlen>=1) {
    id = static_cast<unsigned int>(retbuf[0]);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putBallLocation (const BallLocation& src) throw (std::bad_alloc) {
  char buffer [bufferSizeBallLocation(src)];
  unsigned int len = encodeBallLocation (buffer, src);
  return socket.put (4,buffer,len);
}
bool TribotsUDPCommunication::getBallLocation (BallLocation& dest) throw () {
  return socket.get (4,retbuf,retlen) && decodeBallLocation (retbuf, retlen, dest);
}

bool TribotsUDPCommunication::putObstacleLocation (const ObstacleLocation& src) throw (std::bad_alloc) {
  char buffer [bufferSizeObstacleLocation(src)];
  unsigned int len = encodeObstacleLocation (buffer, src);
  return socket.put (6,buffer,len,1);
}
bool TribotsUDPCommunication::getObstacleLocation (ObstacleLocation& dest) throw (std::bad_alloc) {
  return socket.get (6,retbuf,retlen) && decodeObstacleLocation (retbuf,retlen,dest);
}
bool TribotsUDPCommunication::putObstacleLocationRequest () throw (std::bad_alloc) {
  return socket.put (7,NULL,0);
}
bool TribotsUDPCommunication::getObstacleLocationRequest () throw () {
  return socket.get (7,retbuf,retlen);
}

bool TribotsUDPCommunication::putVisibleObjectList (const vector<VisibleObjectList>& src) throw (std::bad_alloc) {
  char buffer [bufferSizeVisibleObjectList(src)];
  unsigned int len = encodeVisibleObjectList (buffer, src);
  return socket.put (8,buffer,len,1);
}
bool TribotsUDPCommunication::getVisibleObjectList (vector<VisibleObjectList>& dest) throw (std::bad_alloc) {
  return socket.get (8,retbuf,retlen) && decodeVisibleObjectList (retbuf, retlen, dest);
}
bool TribotsUDPCommunication::putVisibleObjectListRequest () throw (std::bad_alloc) {
  return socket.put (9,NULL,0);
}
bool TribotsUDPCommunication::getVisibleObjectListRequest () throw () {
  return socket.get (9,retbuf,retlen);
}

bool TribotsUDPCommunication::putRobotData (const RobotData& src) throw (std::bad_alloc) {
  char buffer [bufferSizeRobotData(src)];
  unsigned int len = encodeRobotData (buffer, src);
  return socket.put (10,buffer,len,1);
}
bool TribotsUDPCommunication::getRobotData (RobotData& dest) throw (std::bad_alloc) {
  return socket.get (10,retbuf,retlen) && decodeRobotData (retbuf,retlen,dest);
}
bool TribotsUDPCommunication::putRobotDataRequest () throw (std::bad_alloc) {
  return socket.put (11,NULL,0);
}
bool TribotsUDPCommunication::getRobotDataRequest () throw () {
  return socket.get (11,retbuf,retlen);
}

bool TribotsUDPCommunication::putGameState (const GameState& src) throw (std::bad_alloc) {
  char buffer [bufferSizeGameState(src)];
  unsigned int len = encodeGameState (buffer, src);
  return socket.put (12,buffer,len);
}
bool TribotsUDPCommunication::getGameState (GameState& dest) throw () {
  return socket.get (12,retbuf,retlen) && decodeGameState(retbuf,retlen,dest);
}

bool TribotsUDPCommunication::putInGame (const bool& src) throw (std::bad_alloc) {
  char buffer[1];
  buffer[0]=src ? 1 : 0;
  return socket.put (14,buffer,1);
}
bool TribotsUDPCommunication::getInGame (bool& dest) throw () {
  // 2 Moeglichkeiten: direkt oder im GameState uebertragen
  if (socket.get (14,retbuf,retlen) && retlen>=1) {
    dest = (retbuf[0]==1);
    return true;
  }
  if (socket.get (12,retbuf,retlen) && retlen>=2) {
    dest = (retbuf[1]==1);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putRefereeState (const RefereeState& src) throw (std::bad_alloc) {
  char buffer[1];
  buffer[0]=src;
  return socket.put (16,buffer,1);
}
bool TribotsUDPCommunication::getRefereeState (RefereeState& dest) throw () {
  // 2 Moeglichkeiten: direkt oder im GameState uebertragen
  if (socket.get (16,retbuf,retlen) && retlen>=1) {
    dest = RefereeState(retbuf[0]);
    return true;
  }
  if (socket.get (12,retbuf,retlen) && retlen>=2) {
    dest = RefereeState (retbuf[0]);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putOwnHalf (const int& src) throw (std::bad_alloc) {
  char buffer [bufferSizeOwnHalf(src)];
  unsigned int len = encodeOwnHalf (buffer, src);
  return socket.put (18,buffer,len);
}
bool TribotsUDPCommunication::getOwnHalf (int& dest) throw () {
  return socket.get (18,retbuf,retlen) && decodeOwnHalf (retbuf, retlen, dest);
}

bool TribotsUDPCommunication::putPlayerType (const std::string& src) throw (std::bad_alloc) {
  return socket.put (20, src.c_str(), src.length(), 0);
}
bool TribotsUDPCommunication::getPlayerType (std::string& dest) throw (std::bad_alloc) {
  if (socket.get (20,retbuf,retlen) && retlen>=0) {
    dest = string (retbuf, retlen);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putPlayerRole (const std::string& src) throw (std::bad_alloc) {
  return socket.put (22, src.c_str(), src.length(), 0);
}
bool TribotsUDPCommunication::getPlayerRole (std::string& dest) throw (std::bad_alloc) {
  if (socket.get (22,retbuf,retlen) && retlen>=0) {
    dest = string (retbuf, retlen);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putPlayerTypeList (const std::vector<std::string>& src) throw (std::bad_alloc) {
  char buffer [bufferSizeStringList (src)];
  unsigned int len = encodeStringList (buffer, src);
  return socket.put (24, buffer, len);
}
bool TribotsUDPCommunication::getPlayerTypeList (std::vector<std::string>& dest) throw (std::bad_alloc) {
  return socket.get (24,retbuf,retlen) && decodeStringList (retbuf, retlen, dest);
}
bool TribotsUDPCommunication::putPlayerTypeListRequest () throw (std::bad_alloc) {
  return socket.put (25,NULL,0);
}
bool TribotsUDPCommunication::getPlayerTypeListRequest () throw () {
  return socket.get (25,retbuf,retlen);
}

bool TribotsUDPCommunication::putPlayerRoleList (const std::vector<std::string>& src) throw (std::bad_alloc) {
  char buffer [bufferSizeStringList (src)];
  unsigned int len = encodeStringList (buffer, src);
  return socket.put (26, buffer, len);
}
bool TribotsUDPCommunication::getPlayerRoleList (std::vector<std::string>& dest) throw (std::bad_alloc) {
  return socket.get (26,retbuf,retlen) && decodeStringList (retbuf, retlen, dest);
}
bool TribotsUDPCommunication::putPlayerRoleListRequest () throw (std::bad_alloc) {
  return socket.put (27,NULL,0);
}
bool TribotsUDPCommunication::getPlayerRoleListRequest () throw () {
  return socket.get (27,retbuf,retlen);
}

bool TribotsUDPCommunication::putFieldGeometry (const FieldGeometry& src) throw (std::bad_alloc) {
  char buffer [bufferSizeFieldGeometry (src)];
  unsigned int len = encodeFieldGeometry (buffer, src);
  return socket.put (28,buffer,len);
}
bool TribotsUDPCommunication::getFieldGeometry (FieldGeometry& dest) throw () {
  return socket.get (28,retbuf,retlen) && decodeFieldGeometry (retbuf, retlen, dest);
}
bool TribotsUDPCommunication::putFieldGeometryRequest () throw (std::bad_alloc) {
  return socket.put (29,NULL,0);
}
bool TribotsUDPCommunication::getFieldGeometryRequest () throw () {
  return socket.get (29,retbuf,retlen);
}

bool TribotsUDPCommunication::putSLHint (const Vec& src1, const Angle& src2) throw (std::bad_alloc) {
  char buffer [bufferSizeSLHint(src1,src2)];
  unsigned int len = encodeSLHint (buffer,src1,src2);
  return socket.put (30,buffer,len);
}
bool TribotsUDPCommunication::getSLHint (Vec& dest1, Angle& dest2) throw () {
  return socket.get (30,retbuf,retlen) && decodeSLHint (retbuf,retlen,dest1,dest2);
}

bool TribotsUDPCommunication::putMessageBoard (const std::vector<std::string>& src) throw (std::bad_alloc) {
  char buffer [bufferSizeStringList(src)];
  unsigned int len=encodeStringList(buffer,src);
  return socket.put (32, buffer, len);
}
bool TribotsUDPCommunication::getMessageBoard (std::vector<std::string>& dest) throw (std::bad_alloc) {
  bool success = false;
  dest.clear();
  std::vector<std::string> dest1;
  for (unsigned int i=0; i<socket.num_messages (32); i++) {
    dest1.clear();
    success |= socket.get (32,retbuf,retlen,i); // die einzelnen empfangenen Nachrichten rueckwaerts durchgehen (das neueste zuerst)
    if (retlen>0) {
      success &= decodeStringList (retbuf,retlen,dest1);
      dest.insert (dest.end(), dest1.begin(), dest1.end());
    }
  }
  return success;
}
bool TribotsUDPCommunication::putMessageBoardRequest () throw (std::bad_alloc) {
  return socket.put (33,NULL,0);
}
bool TribotsUDPCommunication::getMessageBoardRequest () throw () {
  return socket.get (33,retbuf,retlen);
}

bool TribotsUDPCommunication::putVcc (const float& src) throw (std::bad_alloc) {
  char buffer [bufferSizeVcc(src)];
  unsigned int len = encodeVcc (buffer, src);
  return socket.put (34,buffer,len);
}
bool TribotsUDPCommunication::getVcc (float& dest) throw () {
  return socket.get (34,retbuf,retlen) && decodeVcc(retbuf, retlen, dest);
}

bool TribotsUDPCommunication::putStandardRequest () throw (std::bad_alloc) {
  return socket.put (37,NULL,0);
}
bool TribotsUDPCommunication::getStandardRequest () throw () {
  return socket.get (37,retbuf,retlen);
}

bool TribotsUDPCommunication::putSynchronisationSignal (const unsigned short int& src) throw (std::bad_alloc) {
  char buffer [2];
  buffer[0] = static_cast<char>(static_cast<unsigned char>(src/256));
  buffer[1] = static_cast<char>(static_cast<unsigned char>(src%256));
  return socket.put (252,buffer,2);
}

bool TribotsUDPCommunication::getSynchronisationSignal (unsigned short int& dest) throw () {
  if (socket.get (252,retbuf,retlen) && retlen>=2) {
    dest = 256*static_cast<unsigned short int>(static_cast<unsigned char>(retbuf[0]))+static_cast<unsigned short int>(static_cast<unsigned char>(retbuf[1]));
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putTacticsBoard (const TacticsBoard& src) throw (std::bad_alloc) {
  char buffer [bufferSizeTacticsBoard (src)];
  unsigned int len = encodeTacticsBoard (buffer, src);
  return socket.put (42, buffer, len);
}
bool TribotsUDPCommunication::getTacticsBoard (TacticsBoard& dest) throw (std::bad_alloc) {
  dest.clear();
  return socket.get (42,retbuf,retlen) && decodeTacticsBoard (retbuf, retlen, dest);
}

bool TribotsUDPCommunication::putDebugImageRequest () throw (std::bad_alloc) {
  return socket.put (45,NULL,0);
}
bool TribotsUDPCommunication::getDebugImageRequest () throw () {
  return socket.get (45,retbuf,retlen);
}

bool TribotsUDPCommunication::putScore (unsigned int own, unsigned int opponent, unsigned int yellow) throw (std::bad_alloc) {
  char buffer [bufferSizeScore(own,opponent,yellow)];
  unsigned int len=encodeScore(buffer,own,opponent,yellow);
  return socket.put (48,buffer,len);
}
bool TribotsUDPCommunication::getScore (unsigned int& own, unsigned int& opponent, unsigned int& yellow) throw () {
  return socket.get (48,retbuf,retlen) && decodeScore (retbuf,retlen,own,opponent,yellow);
}

bool TribotsUDPCommunication::putDriveVector (const DriveVector& src) throw (std::bad_alloc) {
  char buffer [7];
  write_signed_short (buffer, 1000*src.vtrans.x);
  write_signed_short (buffer+2, 1000*src.vtrans.y);
  write_signed_short (buffer+4, 1000*src.vrot);
  buffer[7]=(src.kick ? static_cast<char>(src.klength) : 0);
  return socket.put (50, buffer, 7);
}
bool TribotsUDPCommunication::getDriveVector (DriveVector& dest) throw () {
  if (socket.get (50,retbuf,retlen) && retlen>=7) {
    dest.vtrans.x = read_signed_short (retbuf);
    dest.vtrans.y = read_signed_short (retbuf+2);
    dest.vrot = read_signed_short (retbuf+4);
    dest.kick = (retbuf[6]!=0);
    dest.klength = static_cast<unsigned char>(retbuf[6]);
    return true;
  }
  return false;
}

bool TribotsUDPCommunication::putSLMirrorHint (const Vec& v) throw (std::bad_alloc) {
  char buffer [4];
  write_signed_short (buffer, v.x);
  write_signed_short (buffer+2, v.y);
  return socket.put (52, buffer, 4);
}
bool TribotsUDPCommunication::getSLMirrorHint (Vec& v) throw () {
  if (socket.get (52,retbuf,retlen) && retlen>=4) {
    v.x = read_signed_short (retbuf);
    v.y = read_signed_short (retbuf+2);
    return true;
  }
  return false;
}


bool TribotsUDPCommunication::putTacticsBoardForce (const TacticsBoard& src) throw (std::bad_alloc) {
  char buffer [bufferSizeTacticsBoard (src)];
  unsigned int len = encodeTacticsBoard (buffer, src);
  return socket.put (56, buffer, len);
}
bool TribotsUDPCommunication::getTacticsBoardForce (TacticsBoard& dest) throw (std::bad_alloc) {
  dest.clear();
  return socket.get (56,retbuf,retlen) && decodeTacticsBoard (retbuf, retlen, dest);
}

