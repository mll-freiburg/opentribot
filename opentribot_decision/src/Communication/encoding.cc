
#include "encoding.h"
#include "NonspecificTaggedUDPCommunication.h"
#include "cstring"
using namespace Tribots;
using namespace std;

unsigned int Tribots::bufferSizeRobotLocation (const RobotLocation&, const TeammateOccupancyGrid&) throw () {
  return 15;
}
unsigned int Tribots::encodeRobotLocation (char* buffer, const RobotLocation& src1, const TeammateOccupancyGrid& src2) throw () {
  NonspecificTaggedUDPCommunication::write_signed_short (buffer, src1.pos.x);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+2, src1.pos.y);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+4, 1000*src1.heading.get_rad());
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+6, 1000*src1.vtrans.x);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+8, 1000*src1.vtrans.y);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+10, 1000*src1.vrot);
  unsigned char mb = (src1.kick ? 1 : 0); // Bit 0 fuer kick, Bit 1 reserviert fuer stark/schwach (zukuenftige Erweiterung)
  mb|=(src1.stuck.robot_stuck ? 4 : 0); // Bit 2 fuer stuck
  mb|=(!src1.valid ? 8 : 0); // Bit 3 fuer delokalisiert, Bits 4..7 frei
  buffer[12]=static_cast<char>(mb);
  unsigned int allcells=0;
  for (unsigned int i=0; i<16; i++)
    allcells = (allcells<<1)+(src2.cells[i]?1:0);
  buffer[13]=static_cast<char>(static_cast<unsigned char>(allcells));
  buffer[14]=static_cast<char>(static_cast<unsigned char>(allcells>>8));
  return 15;
}
bool Tribots::decodeRobotLocation (const char* retbuf, unsigned int retlen, RobotLocation& dest1, TeammateOccupancyGrid& dest2) throw () {
  if (retlen<15)
    return false;
  dest1.pos.x = NonspecificTaggedUDPCommunication::read_signed_short (retbuf);
  dest1.pos.y = NonspecificTaggedUDPCommunication::read_signed_short (retbuf+2);
  dest1.heading.set_rad (0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+4));
  dest1.vtrans.x = 0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+6);
  dest1.vtrans.y = 0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+8);
  dest1.vrot = 0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+10);
  unsigned char mb = static_cast<unsigned char>(retbuf[12]);
  dest1.kick = (mb&1);
  dest1.stuck.robot_stuck = (mb&4);
  dest1.valid = !(mb&8);
  unsigned int og = static_cast<unsigned char> (retbuf[13])+(static_cast<unsigned int>(static_cast<unsigned char> (retbuf[14]))<<8);
  for (unsigned int i=0; i<16; i++) {
    dest2.cells[i] = (og&1);
    og >>= 1;
  }
  return true;
}

unsigned int Tribots::bufferSizeBallLocation (const BallLocation&) throw () {
  return 13;
}
unsigned int Tribots::encodeBallLocation (char* buffer, const BallLocation& src) throw () {
  NonspecificTaggedUDPCommunication::write_signed_short (buffer, src.pos.x);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+2, src.pos.y);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+4, src.pos.z);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+6, 1000*src.velocity.x);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+8, 1000*src.velocity.y);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+10, 1000*src.velocity.z);
  buffer[12]=src.pos_known | (static_cast<unsigned int>(src.velocity_known)<<3);
  return 13;
}
bool Tribots::decodeBallLocation (const char* retbuf, unsigned int retlen, BallLocation& dest) throw () {
  if (retlen<13)
    return false;
  dest.pos.x = NonspecificTaggedUDPCommunication::read_signed_short (retbuf);
  dest.pos.y = NonspecificTaggedUDPCommunication::read_signed_short (retbuf+2);
  dest.pos.z = NonspecificTaggedUDPCommunication::read_signed_short (retbuf+4);
  dest.velocity.x = 0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+6);
  dest.velocity.y = 0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+8);
  dest.velocity.z = 0.001*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+10);
  dest.pos_known=BallLocation::BallAttribute (retbuf[12]&0x03);
  dest.velocity_known=((retbuf[12]>>3)&0x01);
  return true;
}

unsigned int Tribots::bufferSizeObstacleLocation (const ObstacleLocation& src) throw () {
  return (src.size()==0 ? 1 : 5*src.size());
}
unsigned int Tribots::encodeObstacleLocation (char* buffer, const ObstacleLocation& src) throw () {
  char* pt=buffer;
  if (src.size()==0) {
    *pt++=0;
    return 1;
  }
  for (unsigned int i=0; i<src.size(); i++) {
    NonspecificTaggedUDPCommunication::write_signed_short (pt, src[i].pos.x);
    NonspecificTaggedUDPCommunication::write_signed_short (pt+2, src[i].pos.y);
    pt[4]=static_cast<char>(static_cast<unsigned char>(src[i].width/30));
    pt+=5;
  }
  return 5*src.size();
}
bool Tribots::decodeObstacleLocation (const char* retbuf, unsigned int retlen, ObstacleLocation& dest) throw () {
  if (retlen==0)
    return false;
  if (retlen==1) {
    dest.clear();
    return true;
  }
  unsigned int num = retlen/5;
  dest.resize (num);
  const char* pt=retbuf;
  for (unsigned int i=0; i<num; i++) {
    dest[i].pos.x = NonspecificTaggedUDPCommunication::read_signed_short (pt);
    dest[i].pos.y = NonspecificTaggedUDPCommunication::read_signed_short (pt+2);
    dest[i].width = 30*static_cast<double>(static_cast<unsigned char>(pt[4]));
    dest[i].player=-1;
    dest[i].velocity=Vec(0,0);
    pt+=5;
  }
  return true;
}

unsigned int Tribots::bufferSizeVisibleObjectList (const std::vector<VisibleObjectList>& src) throw () {
  unsigned int num = src.size();
  for (unsigned int i=0; i<src.size(); i++)
    num+=src[i].objectlist.size();
  return 6*num;
}
unsigned int Tribots::encodeVisibleObjectList (char* buffer, const std::vector<VisibleObjectList>& src) throw () {
  char* pt=buffer;
  unsigned int buflen=0;
  for (unsigned int j=0; j<src.size(); j++) {
    // Bildquellen-ID schreiben
    NonspecificTaggedUDPCommunication::write_signed_short (pt, 0.0);
    NonspecificTaggedUDPCommunication::write_signed_short (pt+2, 0.0);
    pt[4]=static_cast<char>(127);    // statt ObjectType Kennung fuer neue Bildquelle
    pt[5]=static_cast<char>(j);
    pt+=6;
    for (unsigned int i=0; i<src[j].objectlist.size(); i++) {
      NonspecificTaggedUDPCommunication::write_signed_short (pt, src[j].objectlist[i].pos.x);
      NonspecificTaggedUDPCommunication::write_signed_short (pt+2, src[j].objectlist[i].pos.y);
      pt[4]=static_cast<char>(src[j].objectlist[i].object_type);
      if (src[j].objectlist[i].object_type==VisibleObject::ball3d)
        pt[5]=static_cast<char>(static_cast<unsigned char>(src[j].objectlist[i].z/30));
      else
        pt[5]=static_cast<char>(static_cast<unsigned char>(src[j].objectlist[i].width/30));
      pt+=6;
    }
    buflen+=6+src[j].objectlist.size()*6;
  }
  return buflen;
}
bool Tribots::decodeVisibleObjectList (const char* retbuf, unsigned int retlen, std::vector<VisibleObjectList>& dest) throw () {
  if (retlen==0)
    return false;
  unsigned int num = retlen/6;
  dest.clear();
  const char* pt=retbuf;
  VisibleObject vob;
  for (unsigned int i=0; i<num; i++) {
    if (pt[4]==127) {
      // neue Bildquelle
      VisibleObjectList vol;
      dest.push_back (vol);
    } else {
      // VisibleObject lesen
      vob.pos.x = NonspecificTaggedUDPCommunication::read_signed_short (pt);
      vob.pos.y = NonspecificTaggedUDPCommunication::read_signed_short (pt+2);
      vob.object_type = VisibleObject::ObjectType (pt[4]);
      if (vob.object_type==VisibleObject::ball3d) {
        vob.width=0;
        vob.z = 30*static_cast<double>(static_cast<unsigned char>(pt[5]));
      } else {
        vob.width = 30*static_cast<double>(static_cast<unsigned char>(pt[5]));
        vob.z=0;
      }
      if (dest.size()>0)
        dest[dest.size()-1].objectlist.push_back (vob);
    }
    pt+=6;
  }
  return true;
}

unsigned int Tribots::bufferSizeRobotData (const RobotData& src) throw () {
  return 19+strlen (src.robotIdString);
}
unsigned int Tribots::encodeRobotData (char* buffer, const RobotData& src) throw () {
  buffer[0]=src.BoardID;
  buffer[1]=(src.motors_on ? 1 : 0);
  buffer[2]=static_cast<char>(5*src.wheel_vel[0]);
  buffer[3]=static_cast<char>(5*src.wheel_vel[1]);
  buffer[4]=static_cast<char>(5*src.wheel_vel[2]);
  buffer[5]=static_cast<char>(20*src.robot_vel[0]);
  buffer[6]=static_cast<char>(20*src.robot_vel[1]);
  buffer[7]=static_cast<char>(10*src.robot_vel[2]);
  buffer[8]=static_cast<char>(static_cast<unsigned char>(40*src.motor_current[0]));
  buffer[9]=static_cast<char>(static_cast<unsigned char>(40*src.motor_current[1]));
  buffer[10]=static_cast<char>(static_cast<unsigned char>(40*src.motor_current[2]));
  buffer[11]=static_cast<char>(src.motor_output[0]);
  buffer[12]=static_cast<char>(src.motor_output[1]);
  buffer[13]=static_cast<char>(src.motor_output[2]);
  buffer[14]=(src.motor_temp_switch[0] ? 4 : 0)+(src.motor_temp_switch[1] ? 2 : 0)+(src.motor_temp_switch[2] ? 1 : 0);
  buffer[15]=static_cast<char>(static_cast<unsigned char>(src.motor_temp[0]));
  buffer[16]=static_cast<char>(static_cast<unsigned char>(src.motor_temp[1]));
  buffer[17]=static_cast<char>(static_cast<unsigned char>(src.motor_temp[2]));
  buffer[18]=static_cast<char>(static_cast<unsigned char>(5*src.motor_vcc));
  memcpy (buffer+19, src.robotIdString, strlen (src.robotIdString));
  return 19+strlen (src.robotIdString);
}
bool Tribots::decodeRobotData (const char* retbuf, unsigned int retlen, RobotData& dest) throw () {
  if (retlen<19)
    return false;
  dest.BoardID=retbuf[0];
  dest.motors_on=(retbuf[1]==1);
  dest.wheel_vel[0]=0.2*retbuf[2];
  dest.wheel_vel[1]=0.2*retbuf[3];
  dest.wheel_vel[2]=0.2*retbuf[4];
  dest.robot_vel[0]=0.05*retbuf[5];
  dest.robot_vel[1]=0.05*retbuf[6];
  dest.robot_vel[2]=0.05*retbuf[7];
  dest.motor_current[0]=0.025*static_cast<unsigned char>(retbuf[8]);
  dest.motor_current[1]=0.025*static_cast<unsigned char>(retbuf[9]);
  dest.motor_current[2]=0.025*static_cast<unsigned char>(retbuf[10]);
  dest.motor_output[0]=retbuf[11];
  dest.motor_output[1]=retbuf[12];
  dest.motor_output[2]=retbuf[13];
  dest.motor_temp_switch[0]=retbuf[14]&4;
  dest.motor_temp_switch[1]=retbuf[14]&2;
  dest.motor_temp_switch[2]=retbuf[14]&1;
  dest.motor_temp[0]=static_cast<unsigned char>(retbuf[15]);
  dest.motor_temp[1]=static_cast<unsigned char>(retbuf[16]);
  dest.motor_temp[2]=static_cast<unsigned char>(retbuf[17]);
  dest.motor_vcc=0.2*static_cast<unsigned char>(retbuf[18]);
  memcpy (dest.robotIdString, retbuf+19, retlen-19);
  return true;
}

unsigned int Tribots::bufferSizeGameState (const GameState&) throw () {
  return 2;
}
unsigned int Tribots::encodeGameState (char* buffer, const GameState& src) throw () {
  buffer[0]=src.refstate;
  buffer[1]=(src.in_game  ? 1 : 0);
  return 2;
}
bool Tribots::decodeGameState (const char* retbuf, unsigned int retlen, GameState& dest) throw () {
  if (retlen<2)
    return false;
  dest.refstate = RefereeState (retbuf[0]);
  dest.in_game = (retbuf[1]==1);
  return true;
}

unsigned int Tribots::bufferSizeOwnHalf (const int&) throw () {
  return 1;
}
unsigned int Tribots::encodeOwnHalf (char* buffer, const int& src) throw () {
  buffer[0]=src;
  return 1;
}
bool Tribots::decodeOwnHalf (const char* retbuf, unsigned int retlen, int& dest) throw () {
  if (retlen<1)
    return false;
  dest = retbuf[0];
  return true;
}

unsigned int Tribots::bufferSizeStringList (const std::vector<std::string>& src) throw () {
  unsigned int bufsize = 0;
  for (unsigned int i=0; i<src.size(); i++)
    bufsize+=src[i].length()+1;
  return bufsize;
}
unsigned int Tribots::encodeStringList (char* buffer, const std::vector<std::string>& src) throw () {
  unsigned int len=0;
  char* pt = buffer;
  for (unsigned int i=0; i<src.size(); i++) {
    memcpy (pt, src[i].c_str(), src[i].length());
    pt+=src[i].length();
    (*pt) = '\n';
    pt++;
    len+=src[i].length()+1;
  }
  return len;
}
bool Tribots::decodeStringList (const char* retbuf, unsigned int retlen, std::vector<std::string>& dest) throw () {
  const char* pt=retbuf;
  const char* pt2=retbuf;
  while (pt2<retbuf+retlen) {
    if ((*pt2)=='\n') {
      string newstring (pt, pt2-pt);
      dest.push_back (newstring);
      pt=pt2=pt2+1;
    } else
      pt2++;
  }
  return true;
}

unsigned int Tribots::bufferSizeFieldGeometry (const FieldGeometry& src) throw () {
  return src.serialize().length();
}
unsigned int Tribots::encodeFieldGeometry (char* buffer, const FieldGeometry& src) throw () {
  string s = src.serialize();
  std::strncpy (buffer, s.c_str(), s.length());
  return s.length();
}
bool Tribots::decodeFieldGeometry (const char* retbuf, unsigned int retlen, FieldGeometry& dest) throw () {
  string s (retbuf, retlen);
  return dest.deserialize (s);
}

unsigned int Tribots::bufferSizeSLHint (const Vec&, const Angle&) throw () {
  return 6;
}
unsigned int Tribots::encodeSLHint (char* buffer, const Vec& src1, const Angle& src2) throw () {
  NonspecificTaggedUDPCommunication::write_signed_short (buffer, src1.x);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+2, src1.y);
  NonspecificTaggedUDPCommunication::write_signed_short (buffer+4, 20*src2.get_rad());
  return 6;
}
bool Tribots::decodeSLHint (const char* retbuf, unsigned int retlen, Vec& dest1, Angle& dest2) throw () {
  if (retlen<6)
    return false;
  dest1.x=NonspecificTaggedUDPCommunication::read_signed_short (retbuf);
  dest1.y=NonspecificTaggedUDPCommunication::read_signed_short (retbuf+2);
  dest2.set_rad (0.05*NonspecificTaggedUDPCommunication::read_signed_short (retbuf+4));
  return true;
}

unsigned int Tribots::bufferSizeVcc (const float&) throw () {
  return 1;
}
unsigned int Tribots::encodeVcc (char* buffer, const float& src) throw () {
  double src2=src;
  buffer[0]=static_cast<char>(static_cast<unsigned char>(5*src2));
  return 1;
}
bool Tribots::decodeVcc (const char* retbuf, unsigned int retlen, float& dest) throw () {
  if (retlen<1)
    return false;
  dest = 0.2*static_cast<unsigned char>(retbuf[0]);
  return true;
}

unsigned int Tribots::bufferSizeTacticsBoard (const TacticsBoard& src) throw () {
  unsigned int bufsize = 0;
  map<string, string>::const_iterator it;
  for (it=src.begin(); it!=src.end(); it++)
    bufsize+=2+it->first.length()+it->second.length();
  return bufsize;
}
unsigned int Tribots::encodeTacticsBoard (char* buffer, const TacticsBoard& src) throw () {
  unsigned int bufsize = 0;
  char* ptr = buffer;
  map<string, string>::const_iterator it;
  for (it=src.begin(); it!=src.end(); it++) {
    const string& key (it->first);
    const string& value (it->second);
    (*(ptr++)) = static_cast<char>(static_cast<unsigned char>(key.length()));
    (*(ptr++)) = static_cast<char>(static_cast<unsigned char>(value.length()));
    memcpy (ptr, key.c_str(), key.length());
    ptr+=key.length();
    memcpy (ptr, value.c_str(), value.length());
    ptr+=value.length();
    bufsize+=2+key.length()+value.length();
  }
  return bufsize;
}
bool Tribots::decodeTacticsBoard (const char* retbuf, unsigned int retlen, TacticsBoard& dest) throw () {
  const char* ptr=retbuf;
  while (ptr+2<retbuf+retlen) {
    unsigned int keylen = static_cast<unsigned int>(static_cast<unsigned char>(*ptr));
    unsigned int valuelen = static_cast<unsigned int>(static_cast<unsigned char>(*(ptr+1)));
    if (ptr+2+keylen+valuelen<=retbuf+retlen) {
      string key = string (ptr+2, keylen);
      string value = string (ptr+2+keylen, valuelen);
      dest[key]=value;
    }
    ptr+=2+keylen+valuelen;
  }
  return true;
}

unsigned int Tribots::bufferSizeScore (unsigned int, unsigned int, unsigned int) throw () {
  return 2;
}
unsigned int Tribots::encodeScore (char* buffer, unsigned int own, unsigned int opponent, unsigned int yellow) throw () {
  if (own>31) own=31;
  if (opponent>31) opponent=31;
  if (yellow>7) yellow=7;
  unsigned int msg = (own<<11)+(opponent<<6)+(yellow<<3);  // Bit 11..15: eigeneTore, Bit 6..10: gegnerTore, Bit 3..5: gelbe Karten, Bit 0..2: frei
  buffer[0] = static_cast<char>(static_cast<unsigned char>(msg>>8));
  buffer[1] = static_cast<char>(static_cast<unsigned char>(msg&255));
  return 2;
}
bool Tribots::decodeScore (const char* retbuf, unsigned int retlen, unsigned int& own, unsigned int& opponent, unsigned int& yellow) throw () {
  if (retlen<2)
    return false;
  unsigned int msg = (static_cast<unsigned short int>(static_cast<unsigned char>(retbuf[0]))<<8)+static_cast<unsigned short int>(static_cast<unsigned char>(retbuf[1]));
  own = (msg>>11);
  opponent = ((msg>>6)&31);
  yellow = ((msg>>3)&7);
  return true;
}
