#include "RandomChoiceBehavior.h"
#include <iostream>
#include "../Fundamental/random.h"
#include "../WorldModel/WorldModel.h"

using namespace std;
using namespace Tribots;

RandomChoiceBehavior::RandomChoiceBehavior(string name)
  : Behavior(name), intention(-1)
{}

RandomChoiceBehavior::~RandomChoiceBehavior() throw() {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior) {
      delete options[i].behavior;
    }
  }
}

void RandomChoiceBehavior::loseControl(const Time& time) throw(TribotsException) {
  intention=-1;
}

void RandomChoiceBehavior::gainControl(const Time& time) throw(TribotsException) {
  internalRCRandomChoice(time, false);
  if (intention<static_cast<int>(options.size()))
    options[intention].behavior->gainControl(time);
}

DriveVector RandomChoiceBehavior::getCmd(const Time& time) throw(TribotsException) {
  if (intention>=0 && intention<static_cast<int>(options.size())) {
    return options[intention].behavior->getCmd(time);
  }
  LOUT << "WARNING: RandomChoiceBehavior::getCmd() called although no intention was chosen\n";
  DriveVector null (Vec(0,0),0,false);
  return null;
}

void RandomChoiceBehavior::addOption (Behavior* b, double p, bool own) throw () {
  RandomOption newOption;
  newOption.behavior=b;
  newOption.probability=(p<0 ? 0 : p);
  newOption.ownBehavior=own;
  options.push_back (newOption);
  Time now;
}

void RandomChoiceBehavior::changeProbability (unsigned int i, double p) throw () {
  if (i<options.size()) {
    options[i].probability=(p<0 ? 0: p);
  }
}

void RandomChoiceBehavior::internalRCRandomChoice (const Time & t, bool blind) throw () {
  intention=0;
  double sum=0;
  for (unsigned int i=0; i<options.size(); i++) {
    if (blind || options[i].behavior->checkInvocationCondition (t))
      options[i].cprobability=options[i].probability;
    else
      options[i].cprobability=0;
    sum+=options[i].cprobability;
  }
  double rnd = 0;
  if (sum>0)
    rnd=urandom (0,sum);
  double psum=0;
  for (unsigned int i=0; i<options.size(); i++) {
    psum+=options[i].cprobability;
    if (rnd<psum) {
      intention=i;
      return;
    }
  }
}

bool RandomChoiceBehavior::checkInvocationCondition(const Time& t) throw() {
  if (intention>=0 && intention<static_cast<int>(options.size()))
    return options[intention].behavior->checkInvocationCondition (t);
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].behavior->checkInvocationCondition (t))
      return true;  // wenn keine Intention gesetzt ist, true liefern wenn zumindest eine Option rechnen koennte
  }
  return false;
}

bool RandomChoiceBehavior::checkCommitmentCondition(const Time& t) throw() {
  if (intention>=0 && intention<static_cast<int>(options.size()))
    return options[intention].behavior->checkCommitmentCondition (t);
  return false;
}

void RandomChoiceBehavior::printHierarchy (std::ostream& os, unsigned int level) const throw () {
  for (unsigned int i=0; i<level; i++)
    os << "  ";
  os << getName() << " (RandomChoice)\n";
  for (unsigned int i=0; i<options.size(); i++)
    options[i].behavior->printHierarchy (os, level+1);
}

void RandomChoiceBehavior::updateTactics (const TacticsBoard& tb) throw () {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior)
      options[i].behavior->updateTactics (tb);
  }
}

void RandomChoiceBehavior::setPlayerRole (const char* role) throw () {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior)
      options[i].behavior->setPlayerRole(role);
  }
}

std::string RandomChoiceBehavior::getIntentionName () const throw () {
  std::string dest = name+"::";
  if (intention>=0 && intention<static_cast<int>(options.size()))
    dest+=options[intention].behavior->getIntentionName();
  else
    dest+="NONE";
  return dest;
}
