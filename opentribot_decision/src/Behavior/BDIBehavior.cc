#include "BDIBehavior.h"
#include "../WorldModel/WorldModel.h"
#include "../Structures/Journal.h"
#include <iostream>

using namespace std;
using namespace Tribots;

BDIBehavior::BDIBehavior(string name)
  : Behavior(name),
    presentIntention(-1), scheduledIntention(-1)
{}

BDIBehavior::~BDIBehavior() throw() {
  for (unsigned int i=0; i < options.size(); i++) {
    if (options[i].ownBehavior) {
      delete options[i].behavior;
    }
  }
}


void BDIBehavior::cycleCallBack(const Time&) throw() {
  scheduledIntention=-1;
}

void BDIBehavior::loseControl(const Time& time) throw(TribotsException) {
  if (presentIntention>=0 && presentIntention<static_cast<int>(options.size())) {
    options[presentIntention].behavior->loseControl(time);
  }
  presentIntention=scheduledIntention=-1;
}

bool BDIBehavior::internalBDICheckConditions (const Time& time) throw () {
  internalBDIUpdateIntention(time);
  return (scheduledIntention>=0 && scheduledIntention<static_cast<int>(options.size()));
}

void BDIBehavior::internalBDIUpdateIntention (const Time& time) throw () {
  scheduledIntention=options.size();  // bedeutet: keine Option auswaehlbar
  if (presentIntention>=0 && presentIntention<static_cast<int>(options.size()) && options[presentIntention].sticky) {  // lazy evaluation sei Dank
    if (options[presentIntention].behavior->checkCommitmentCondition(time))
      scheduledIntention=presentIntention;  // Im stickyMode die Intention nicht veraendern, falls bereits gesetzt
    return;
  }
  bool wantContinue=false;
  intentionFinished=true;
  if (presentIntention>=0 && presentIntention<static_cast<int>(options.size())) {
    if (options[presentIntention].behavior->checkCommitmentCondition(time)) {
      wantContinue=true;
      intentionFinished=false;
      scheduledIntention=presentIntention;
      if (!options[presentIntention].allowInterrupt)
        return;
    }
  }
  int maxCheck = (wantContinue ? presentIntention : options.size());
  for (int i=0; i<maxCheck; i++) {
    if (options[i].behavior->checkInvocationCondition(time)) {
      scheduledIntention=i;
      break;
    }
  }
}

DriveVector BDIBehavior::getCmd(const Time& time) throw(TribotsException) {
  if (scheduledIntention<0)
    internalBDIUpdateIntention(time);
  DriveVector dv (Vec(0,0), 0, false);
  if (presentIntention!=scheduledIntention || intentionFinished) {
    if (presentIntention>=0 && presentIntention<static_cast<int>(options.size())) {  // BDI war bereits aktiv
      options[presentIntention].behavior->loseControl(time);
      if (scheduledIntention<static_cast<int>(options.size())) {  // BDI weiterhin aktiv
        options[scheduledIntention].behavior->gainControl(time);
      }
    } else if (scheduledIntention>=0 && scheduledIntention<static_cast<int>(options.size())) {
      options[scheduledIntention].behavior->gainControl(time);  // BDI bisher inaktiv
    }
  }
  presentIntention=scheduledIntention;
  if (presentIntention<static_cast<int>(options.size())) {
    dv=options[presentIntention].behavior->getCmd(time);
  } else {
    LOUT << "WARNING: BDIBehavior::getCmd() called although no option is executable\n";
  }
  scheduledIntention=-1;
  return dv;
}

bool BDIBehavior::checkInvocationCondition(const Time& time) throw() {
  return internalBDICheckConditions (time);
}

bool BDIBehavior::checkCommitmentCondition(const Time& time) throw() {
  return internalBDICheckConditions (time);
}

void BDIBehavior::printHierarchy (std::ostream& os, unsigned int level) const throw () {
  for (unsigned int i=0; i<level; i++)
    os << "  ";
  os << getName() << " (BDI)\n";
  for (unsigned int i=0; i<options.size(); i++)
    options[i].behavior->printHierarchy (os, level+1);
}

void BDIBehavior::updateTactics (const TacticsBoard& tb) throw () {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior)
      options[i].behavior->updateTactics (tb);
  }
}

void BDIBehavior::setPlayerRole (const char* role) throw () {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior)
      options[i].behavior->setPlayerRole(role);
  }
}

void BDIBehavior::addOption (Behavior* b, bool interrupt, bool sticky, bool own) throw () {
  BDIOption newOption;
  newOption.behavior=b;
  newOption.ownBehavior=own;
  newOption.allowInterrupt=interrupt;
  newOption.sticky=sticky;
  options.push_back (newOption);
}

std::string BDIBehavior::getIntentionName () const throw () {
  std::string dest = name+"::";
  if (presentIntention>=0 && presentIntention<static_cast<int>(options.size()))
    dest+=options[presentIntention].behavior->getIntentionName();
  else
    dest+="NONE";
  return dest;
}
