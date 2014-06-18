#include "SPBehavior.h"
#include "../WorldModel/WorldModel.h"
#include <iostream>

using namespace Tribots;
using namespace std;

SPBehavior::SPBehavior(string name) :
    Behavior(name),
    presentIntention(-1),
    scheduledIntention(-1)
 {}

SPBehavior::~SPBehavior() throw() {
  for (unsigned int i=0; i < options.size(); i++) {
    if (options[i].ownBehavior) {
      delete options[i].behavior;
    }
  }
}

void SPBehavior::cycleCallBack(const Time&) throw() {
  scheduledIntention=-1;
}

void SPBehavior::appendStage (Behavior* b, bool skip, bool interrupt, bool own) throw (std::bad_alloc) {
  Stage newstage;
  newstage.behavior=b;
  newstage.allowSkip=skip;
  newstage.allowInterrupt=interrupt;
  newstage.interruptInfluence=options.size()+10;
  newstage.ownBehavior=own;
  options.push_back (newstage);
}

void SPBehavior::appendStage (Behavior* b, bool skip, bool interrupt, int intval, bool own) throw (std::bad_alloc) {
  Stage newstage;
  newstage.behavior=b;
  newstage.allowSkip=skip;
  newstage.allowInterrupt=interrupt;
  newstage.interruptInfluence=intval;
  newstage.ownBehavior=own;
  options.push_back (newstage);
}

void SPBehavior::loseControl(const Time& time) throw(TribotsException) {
  if (presentIntention>=0 && presentIntention<static_cast<int>(options.size())) {
    options[presentIntention].behavior->loseControl(time);
  }
  presentIntention=scheduledIntention=-1;
}

bool SPBehavior::checkInvocationCondition(const Time& time) throw() {
  return internalSPCheckConditions (time);
}

bool SPBehavior::checkCommitmentCondition(const Time& time) throw() {
  return internalSPCheckConditions (time);
}

bool SPBehavior::internalSPCheckConditions (const Time& time) throw () {
  internalSPUpdateIntention (time);
  return (scheduledIntention>=0 && scheduledIntention<static_cast<int>(options.size()));
}

void SPBehavior::internalSPUpdateIntention (const Time& time) throw() {
  // Iterationen:
  //   scheduledIntention traegt den Index der hoechsten aktivierbaren Stufe
  //   nextStage traegt den Index der hoeschsten erreichbaren Stufe
  //   continueSearch gibt an, ob die aktuelle Stufe uebersprungen werden darf
  const int stageNum = options.size();
  scheduledIntention = stageNum;  // default, falls keine aktivierbare Stufe gefunden wird

  bool interruptAllowed = false;  // darf die aktuelle Stufe unterbrochen werden?
  bool continuationFound =  false;  // wurde bereits eine moegliche Fortsetzung gefunden?
  intentionFinished = true;
  if (presentIntention>=0 && presentIntention<stageNum) {
    interruptAllowed = options[presentIntention].allowInterrupt;
    if (options[presentIntention].behavior->checkCommitmentCondition(time)) {
      continuationFound = true; // wenn gegenwaertige Stufe fortsetzbar ist, diese Stufe merken
      intentionFinished = false;
      scheduledIntention = presentIntention;
    }
  }

  bool continueSearch = !continuationFound || interruptAllowed;  // (koennte man an dieser Stelle auch auf true setzen)
  int nextStage = presentIntention+1;
  while (continueSearch && nextStage<stageNum) {
    bool mightInterrupt = interruptAllowed && nextStage-presentIntention<=options[nextStage].interruptInfluence;
    if ((!continuationFound || mightInterrupt) && options[nextStage].behavior->checkInvocationCondition(time)) {
      scheduledIntention = nextStage; // Stufe merken
      continuationFound = true;  // eine Weiterfuehrung wurde gefunden
      if (!interruptAllowed) {
        return; // (im Prinzip verzichtbar)
      }
    }
    continueSearch = options[nextStage].allowSkip; // nur dann die nachfolgende Stufe untersuchen, wenn aktuelle Stufe ueberspringbar
    nextStage++;
  }
}

DriveVector SPBehavior::getCmd(const Time& time)
    throw(TribotsException)
{
  if (scheduledIntention<0)
    internalSPUpdateIntention (time);
  DriveVector dv (Vec(0,0), 0, false);
  if (presentIntention!=scheduledIntention || intentionFinished) {
    if (presentIntention>=0 && presentIntention<=static_cast<int>(options.size())) {
      options[presentIntention].behavior->loseControl(time);
      if (scheduledIntention<static_cast<int>(options.size())) {
        options[scheduledIntention].behavior->gainControl(time);
      }
    } else if (scheduledIntention>=0 && scheduledIntention<static_cast<int>(options.size())) {
      options[scheduledIntention].behavior->gainControl(time);
    }
  }
  presentIntention=scheduledIntention;
  if (presentIntention<static_cast<int>(options.size())) {
    dv=options[presentIntention].behavior->getCmd(time);
  } else {
    LOUT << "WARNING: SPBehavior::getCmd() called although no stage is executable\n";
  }
  scheduledIntention=-1;
  return dv;
}

void SPBehavior::printHierarchy (std::ostream& os, unsigned int level) const throw () {
  for (unsigned int i=0; i<level; i++)
    os << "  ";
  os << getName() << " (SP)\n";
  for (unsigned int i=0; i<options.size(); i++)
    options[i].behavior->printHierarchy (os, level+1);
}

void SPBehavior::updateTactics (const TacticsBoard& tb) throw () {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior)
      options[i].behavior->updateTactics (tb);
  }
}

void SPBehavior::setPlayerRole (const char* role) throw () {
  for (unsigned int i=0; i<options.size(); i++) {
    if (options[i].ownBehavior)
      options[i].behavior->setPlayerRole(role);
  }
}

std::string SPBehavior::getIntentionName () const throw () {
  std::string dest = name+"::";
  if (presentIntention>=0 && presentIntention<static_cast<int>(options.size()))
    dest+=options[presentIntention].behavior->getIntentionName();
  else
    dest+="NONE";
  return dest;
}
