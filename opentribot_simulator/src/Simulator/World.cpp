
#include "World.h"
#include "helpers.h"
#include <stdexcept>
#include <fstream>
#include <cmath>
#ifndef NO_X
#include <drawstuff/drawstuff.h>
#endif
#include "global.h"

using namespace TribotsSim;
using namespace std;

const dReal* GeneralObject::getPosition () const {
  return dBodyGetPosition (body);
}
      
const dReal* GeneralObject::getLinearVelocity () const {
  return dBodyGetLinearVel (body);
}

const dReal* GeneralObject::getRotation () const {
  return dBodyGetRotation (body);
}

const dReal* GeneralObject::getAngularVelocity () const {
  return dBodyGetAngularVel (body);
}

void GeneralObject::setPosition (dReal x, dReal y, dReal z) {
  dBodySetPosition (body, x, y, z);
}

void GeneralObject::setRotation (dReal z, dReal y, dReal x) {
  dReal dr [3*4];        // TODO: muss noch getestet werden, evtl. muessen x,y,z vertauscht werden
  dr[0]=cos(x)*cos(y);
  dr[1]=cos(x)*sin(y)*sin(z)-sin(x)*cos(z);
  dr[2]=cos(x)*sin(y)*cos(z)+sin(x)*sin(z);
  dr[3]=0;
  dr[4]=sin(x)*cos(y);
  dr[5]=sin(x)*sin(y)*sin(z)+cos(x)*cos(z);
  dr[6]=sin(x)*sin(y)*cos(z)-cos(x)*sin(z);
  dr[7]=0;
  dr[8]=-sin(y);
  dr[9]=cos(y)*sin(z);
  dr[10]=cos(y)*cos(z);
  dr[11]=0;
  dBodySetRotation (body, dr);
}

void GeneralObject::setLinearVelocity (dReal x, dReal y, dReal z) {
  dBodySetLinearVel (body, x, y, z);
}

void GeneralObject::setAngularVelocity (dReal r1, dReal r2, dReal r3) {
  dBodySetAngularVel (body, r1, r2, r3);
}

void GeneralObject::addRelativeForce (dReal f1, dReal f2, dReal f3) {
  dBodyAddRelForce (body, f1, f2, f3);
}

void GeneralObject::addForce (dReal f1, dReal f2, dReal f3) {
  dBodyAddForce (body, f1, f2, f3);
}

void GeneralObject::addRelativeTorque (dReal f1, dReal f2, dReal f3) {
  dBodyAddRelTorque (body, f1, f2, f3);
}

void GeneralObject::addTorque (dReal f1, dReal f2, dReal f3) {
  dBodyAddTorque (body, f1, f2, f3);
}




namespace {

  void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb) {
  #ifndef NO_X
    // Code uebertragen aus Stefan Welkers Simulator
    int i;
    
    if (!g) return;
    if (!pos) pos = dGeomGetPosition (g);
    if (!R) R = dGeomGetRotation (g);
    dsSetColor (0.3,0.3,0.3);
  
    int type = dGeomGetClass (g);
    if (type == dBoxClass) {
      dVector3 sides;
      dGeomBoxGetLengths (g,sides);
      dsDrawBox (pos,R,sides);
    }
    else if (type == dSphereClass) {
      dsDrawSphere (pos,R,dGeomSphereGetRadius (g));
    }
    else if (type == dCCylinderClass) {
      dReal radius,length;
      dGeomCCylinderGetParams (g,&radius,&length);
      dsDrawCappedCylinder (pos,R,length,radius);
    }
  /*
    // cylinder option not yet implemented
    else if (type == dCylinderClass) {
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
  */
    else if (type == dGeomTransformClass) {
      dGeomID g2 = dGeomTransformGetGeom (g);
      const dReal *pos2 = dGeomGetPosition (g2);
      const dReal *R2 = dGeomGetRotation (g2);
      dVector3 actual_pos;
      dMatrix3 actual_R;
      dMULTIPLY0_331 (actual_pos,R,pos2);
      actual_pos[0] += pos[0];
      actual_pos[1] += pos[1];
      actual_pos[2] += pos[2];
      dMULTIPLY0_333 (actual_R,R,R2);
      drawGeom (g2,actual_pos,actual_R,0);
    }
  
    if (show_aabb) {
      // draw the bounding box for this geom
      dReal aabb[6];
      dGeomGetAABB (g,aabb);
      dVector3 bbpos;
      for (i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
      dVector3 bbsides;
      for (i=0; i<3; i++) bbsides[i] = aabb[i*2+1] - aabb[i*2];
      dMatrix3 RI;
      dRSetIdentity (RI);
      dsSetColorAlpha (1,0,0,0.5);
      dsDrawBox (bbpos,RI,bbsides);
    }
  #endif
  }
  
}



Tribot::Tribot () : num_geom (0), is_active (false) {
  setNoSteering();
}

Tribot::~Tribot () {;}

void Tribot::setNoSteering () {
  steering[0]=steering[1]=steering[2]=steering[3]=0;
}
void Tribot::setSteeringX (dReal s) { steering[0]=s; }
void Tribot::setSteeringY (dReal s) { steering[1]=s; }
void Tribot::setSteeringPhi (dReal s) { steering[2]=s; }
void Tribot::setSteeringKick (bool s) { steering[3]=(s ? 1 : 0); }
const dReal* Tribot::getSteering () const { return steering; }

void Tribot::setActive (bool b) {
  is_active=b;
}

bool Tribot::isActive () const {
  return is_active;
}

void Tribot::create (dWorldID world, dSpaceID space) {
  // Code uebertragen aus Stefan Welkers Simulator
  dGeomID g2[4];
        
  dMass m,macc;
  dMassSetZero (&m);

  body = dBodyCreate (world);
  dMassSetBox (&macc,1,0.4,0.4,0.4);
  dMassAdjust (&macc,15);
  dMassAdd (&m,&macc);
  dMassSetBox (&macc,1,0.2,0.1,0.6);
  
  geom[0] = dCreateGeomTransform (space); dGeomTransformSetCleanup (geom[0],1);
  geom[1] = dCreateGeomTransform (space); dGeomTransformSetCleanup (geom[1],1);
  geom[2] = dCreateGeomTransform (space); dGeomTransformSetCleanup (geom[2],1);
  geom[3] = dCreateGeomTransform (space); dGeomTransformSetCleanup (geom[3],1);
  
  dBodySetPosition (body,2,1,1);
  g2[0] = dCreateBox (0,0.3,0.4,0.4);
  g2[1] = dCreateBox (0,0.2,0.1,0.3);
  g2[2] = dCreateBox (0,0.18,0.03,0.2);
  g2[3] = dCreateBox (0,0.18,0.03,0.2);
  dGeomTransformSetGeom (geom[0],g2[0]);
  dGeomTransformSetGeom (geom[1],g2[1]);
  dGeomTransformSetGeom (geom[2],g2[2]);
  dGeomTransformSetGeom (geom[3],g2[3]);
  dGeomSetPosition (g2[0],0.04,0,0);
  dGeomSetPosition (g2[1],-0.2,0,-0.05);
  dGeomSetPosition (g2[2], 0.22,-0.18,-0.1);
  dGeomSetPosition (g2[3], 0.22,+0.18,-0.1);
  dMatrix3 Rtx;
  dRFromAxisAndAngle (Rtx,0,0,0.5,-0.1);
  dGeomSetRotation (g2[2],Rtx);
  dRFromAxisAndAngle (Rtx,0,0,0.5,0.1);
  dGeomSetRotation (g2[3],Rtx);

  dGeomSetBody (geom[0],body);
  dGeomSetBody (geom[1],body);
  dGeomSetBody (geom[2],body);
  dGeomSetBody (geom[3],body);
  
  dBodySetMass (body,&m);

  num_geom=4;
}

void Tribot::draw () {
#ifndef NO_X
  // Code uebertragen aus Stefan Welkers Simulator
  for (unsigned int i=0; i<num_geom; i++){
    drawGeom (geom[i],0,0,0);
  }
  if (is_active) {
    dsSetColor(1,0,0);
    dReal headsides[3] = {0.36*0.9,0.36*0.3f,0.5*1.1f};
    dsDrawBox (dGeomGetPosition(geom[0]),dGeomGetRotation(geom[0]), headsides);
  }
#endif
}

void Tribot::makeSnapshot (std::vector<dReal>& dest) {
  appendVecToVector3 (dest, dBodyGetPosition(body));
  appendVecToVector3 (dest, dBodyGetLinearVel(body));
  appendVecToVector4 (dest, dBodyGetQuaternion(body));
}

unsigned int Tribot::recoverSnapshot (const std::vector<dReal>& src, unsigned int index) {
  if (src.size()<10+index)
    throw std::invalid_argument ("Snapshot unvollstaendig in TribotsSim::Tribot::recoverSnapshot");
  dBodySetPosition (body,src[index], src[index+1], src[index+2]);
  dBodySetLinearVel (body,src[index+3], src[index+4], src[index+5]);
  dQuaternion quat;
  quat[0]=src[index+6];
  quat[1]=src[index+7];
  quat[2]=src[index+8];
  quat[3]=src[index+9];
  dBodySetQuaternion (body,quat);
  return index+10;
}


Ball::Ball () : is_active (false) {;}

Ball::~Ball () {;}

void Ball::create (dWorldID world, dSpaceID space) {
  dMass m;
  dMassSetZero (&m);
  dMassAdjust (&m,0.4);
  body = dBodyCreate(world);
  dMassSetSphere (&m, 10.4, BALL_RADIUS);
  dBodySetMass (body, &m);
  dBodySetPosition (body,0,0,1);
  geom=dCreateSphere (space, BALL_RADIUS);
  dGeomSetBody (geom,body);
}

void Ball::draw () {
#ifndef NO_X
  dsSetColor (1,0.5,0.5);
  if (is_active)
    dsSetColor (1,0.4, 0);
  dsDrawSphere (dGeomGetPosition(geom),dGeomGetRotation(geom),BALL_RADIUS+0.01);
#endif
}

void Ball::setActive (bool b) {
  is_active=b;
}

bool Ball::isActive () const {
  return is_active;
}

void Ball::makeSnapshot (std::vector<dReal>& dest) {
  appendVecToVector3 (dest, dBodyGetPosition(body));
  appendVecToVector3 (dest, dBodyGetLinearVel(body));
  appendVecToVector4 (dest, dBodyGetQuaternion(body));
}

unsigned int Ball::recoverSnapshot (const std::vector<dReal>& src, unsigned int index) {
  if (src.size()<10+index)
    throw std::invalid_argument ("Snapshot unvollstaendig in TribotsSim::Tribot::recoverSnapshot");
  dBodySetPosition (body,src[index], src[index+1], src[index+2]);
  dBodySetLinearVel (body,src[index+3], src[index+4], src[index+5]);
  dQuaternion quat;
  quat[0]=src[index+6];
  quat[1]=src[index+7];
  quat[2]=src[index+8];
  quat[3]=src[index+9];
  dBodySetQuaternion (body,quat);
  return index+10;
}


Goal::Goal () {
  num_geom=0;
}

Goal::~Goal () {;}

void Goal::create (dWorldID, dSpaceID space, int side1) {
  side=side1;
  num_geom=3;
  geom[0]=dCreateBox (space,0.1,2.0,1);
  geom[1]=dCreateBox (space,0.6,0.1,1);
  geom[2]=dCreateBox (space,0.6,0.1,1);
  dGeomSetBody (geom[0],0);
  dGeomSetBody (geom[1],0);
  dGeomSetBody (geom[2],0);
  dGeomSetPosition (geom[0], side*(FIELD_LENGTH/2+0.6), 0, 0.3);
  dGeomSetPosition (geom[1], side*(FIELD_LENGTH/2+0.3), -1.0, 0.3);
  dGeomSetPosition (geom[2], side*(FIELD_LENGTH/2+0.3), 1.0, 0.3);
}

void Goal::draw () {
#ifndef NO_X
  dReal sides0[3] = {0.1,2.2,1};
  dReal sides1[3] = {0.6,0.1,1};
  if (side>0)
    dsSetColor (0,0,1);
  else
    dsSetColor (1,1,0);
  dsDrawBox (dGeomGetPosition(geom[0]),dGeomGetRotation(geom[0]),sides0);
  dsDrawBox (dGeomGetPosition(geom[1]),dGeomGetRotation(geom[1]),sides1);
  dsDrawBox (dGeomGetPosition(geom[2]),dGeomGetRotation(geom[2]),sides1);
#endif
}


Field::Field () {;}
Field::~Field () {;}

void Field::create (dWorldID, dSpaceID) {;}
void Field::draw () {
#ifndef NO_X
  dsSetColor (1,1,1);
  float field_length=FIELD_LENGTH;
  float field_width=FIELD_WIDTH;
  float flh=field_length/2;
  float fwh=field_width/2;
  float lh=0.03;
  double a[3];
  double b[3];
  a[2]=b[2]=lh;
  
  // Seitenlinien:
  a[0]=-flh; a[1]=fwh;
  b[0]=+flh; b[1]=fwh;
  dsDrawLineD(a,b);
  b[0]=-flh; b[1]=-fwh;
  dsDrawLineD(a,b);
  a[0]=flh; a[1]=-fwh;
  dsDrawLineD(a,b);
  b[0]=+flh; b[1]=fwh;
  dsDrawLineD(a,b);
  
  // Mittellinie:
  a[0]=0; a[1]=-fwh;
  b[0]=0; b[1]=fwh;
  dsDrawLineD(a,b);
  
  // Mittelkreis:
  for (unsigned int i=0; i<16; i++) {
    double an1=2*M_PI*i/16.0;
    double an2=2*M_PI*(i+1)/16.0;
    a[0]=CENTER_CIRCLE_RADIUS*cos(an1);
    a[1]=CENTER_CIRCLE_RADIUS*sin(an1);
    b[0]=CENTER_CIRCLE_RADIUS*cos(an2);
    b[1]=CENTER_CIRCLE_RADIUS*sin(an2);
    dsDrawLineD(a,b);
  }
  
  // Goal areas:
  a[0]=-flh; a[1]=-GOAL_AREA_WIDTH/2;
  b[0]=-flh+GOAL_AREA_LENGTH; b[1]=-GOAL_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  a[0]=-flh+GOAL_AREA_LENGTH; a[1]=GOAL_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  b[0]=-flh; b[1]=GOAL_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  
  a[0]=flh; a[1]=-GOAL_AREA_WIDTH/2;
  b[0]=flh-GOAL_AREA_LENGTH; b[1]=-GOAL_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  a[0]=flh-GOAL_AREA_LENGTH; a[1]=GOAL_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  b[0]=flh; b[1]=GOAL_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  
  // Penalty areas:
  a[0]=-flh; a[1]=-PENALTY_AREA_WIDTH/2;
  b[0]=-flh+PENALTY_AREA_LENGTH; b[1]=-PENALTY_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  a[0]=-flh+PENALTY_AREA_LENGTH; a[1]=PENALTY_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  b[0]=-flh; b[1]=PENALTY_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  
  a[0]=flh; a[1]=-PENALTY_AREA_WIDTH/2;
  b[0]=flh-PENALTY_AREA_LENGTH; b[1]=-PENALTY_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  a[0]=flh-PENALTY_AREA_LENGTH; a[1]=PENALTY_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  b[0]=flh; b[1]=PENALTY_AREA_WIDTH/2;
  dsDrawLineD(a,b);
  
  // Ecken:
  a[0]=+flh; a[1]=-fwh+CORNER_ARC_RADIUS;
  b[0]=+flh-CORNER_ARC_RADIUS; b[1]=-fwh;
  dsDrawLineD(a,b);
   
  a[0]=+flh; a[1]=+fwh-CORNER_ARC_RADIUS;
  b[0]=+flh-CORNER_ARC_RADIUS; b[1]=fwh;
  dsDrawLineD(a,b);

  a[0]=-flh; a[1]=-fwh+CORNER_ARC_RADIUS;
  b[0]=-flh+CORNER_ARC_RADIUS; b[1]=-fwh;
  dsDrawLineD(a,b);
   
  a[0]=-flh; a[1]=+fwh-CORNER_ARC_RADIUS;
  b[0]=-flh+CORNER_ARC_RADIUS; b[1]=fwh;
  dsDrawLineD(a,b);

  // Strafstosspunkt:  
  a[0]=flh-PENALTY_AREA_SPOT-0.05; a[1]=0;
  b[0]=flh-PENALTY_AREA_SPOT+0.05; b[1]=0;
  dsDrawLineD(a,b);
  a[0]=-flh+PENALTY_AREA_SPOT-0.05; a[1]=0;
  b[0]=-flh+PENALTY_AREA_SPOT+0.05; b[1]=0;
  dsDrawLineD(a,b);
#endif
}

void World::makeSnapshot (std::vector<dReal>& dest) {
  dest.push_back (1.0);  // Codierungsversion
  dest.push_back (robots.size());
  ball.makeSnapshot (dest);
  for (unsigned int i=0; i<robots.size(); i++)
    robots[i].makeSnapshot (dest);
}

unsigned int World::recoverSnapshot (const std::vector<dReal>& src, unsigned int index) {
  if (src.size()<2+index)
    throw std::invalid_argument ("Snapshot unvollstaendig in TribotsSim::World::recoverSnapshot");
  // src[index] ist Codierungsversion
  unsigned int num_robots = static_cast<unsigned int>(src[index+1]);
  index=ball.recoverSnapshot (src, index+2);
  for (unsigned int i=0; i<num_robots; i++) {
    if (i>=robots.size()) {
      Tribot newtribot;
      newtribot.create (wid, sid);
      robots.push_back (newtribot);
    }
    index=robots[i].recoverSnapshot (src, index);
  }
  return index;
}

void TribotsSim::saveSnapshot (const std::vector<dReal>& array, const char* filename) {
  std::ofstream stream (filename);
  if (!stream)
    throw std::invalid_argument (std::string("Datei ")+filename+"kaputt, in die Schnappschuss gespeichert werden soll");
  for (unsigned int i=0; i<array.size(); i++)
    stream << array[i] << '\n';
  stream << std::flush;
}

void TribotsSim::loadSnapshot (std::vector<dReal>& array, const char* filename) {
  std::ifstream stream (filename);
  if (!stream)
    throw std::invalid_argument (std::string("Datei ")+filename+"kaputt, aus der Schnappschuss geladen werden soll");
  dReal number;
  array.clear();
  while (true) {
    stream >> number;
    if (stream.eof())
      break;
    array.push_back (number);
  }
}

World::World () : wid (dWorldCreate()), sid (dHashSpaceCreate (0)), cid (dJointGroupCreate (0)), gid (dCreatePlane (sid,0,0,1,0)) {
  field.create (wid, sid);
  ball.create (wid, sid);
  Goal g1, g2;
  g1.create (wid, sid, +1);
  g2.create (wid, sid, -1);
  goals.push_back (g1);
  goals.push_back (g2);
  Tribot t1;
  t1.create (wid, sid);
  robots.push_back (t1);
}

World::~World () {
  dJointGroupDestroy (cid);
  dSpaceDestroy (sid);
  dWorldDestroy (wid);
}

void World::draw () {
#ifndef NO_X
  float pos [] = { 0, 0, 0.75*FIELD_LENGTH };
  float look [] = { 90, -90, 0, 0 };
  dsSetViewpoint (pos, look);
#endif

  for (unsigned int i=0; i<goals.size(); i++)
    goals[i].draw();
  ball.draw();
  field.draw ();
  for (unsigned int i=0; i<robots.size(); i++)
    robots[i].draw();
}
