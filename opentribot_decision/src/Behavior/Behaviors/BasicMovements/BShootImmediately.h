#ifndef _BSHOOTIMMEDIATELY_H_
#define _BSHOOTIMMEDIATELY_H_

#include "../../Behavior.h"

namespace Tribots
{

/**
 * Dieses Verhalten l��t sich aufrufen, wenn der Roboter im Ballbesitz ist
 * und dieses Verhalten in den letzten 3 Zyklen nicht aktiv war. Wenn 
 * aktiviert, h�lt das Verhalten die akutelle Bewegung bei und l�st den Kicker
 * sofort aus.
 */
class BShootImmediately : public Tribots::Behavior
{
public:
	BShootImmediately(const std::string name= "BShootImmediately", int hackKickLength =30);
	virtual ~BShootImmediately() throw();
  
  /** siehe invocation condition */
  virtual bool checkCommitmentCondition(const Time&) throw();
  /** 
   * l�st den Kicker aus, falls der Roboter Ballbesitz hat. Das Verhalten
   * l��t sich danach 3 Zyklen nich mehr aktivieren.
   */
  virtual bool checkInvocationCondition(const Time&) throw();

  /** H�lt die aktuelle Bewegung bei und l�st den Kicker aus. */
  virtual DriveVector getCmd(const Time&) throw(TribotsException);
  
protected:
  static long lastActivation; ///< den Kicker zentral blockieren!
  int hackKickLength;
};

};

#endif //_BSHOOTIMMEDIATELY_H_
