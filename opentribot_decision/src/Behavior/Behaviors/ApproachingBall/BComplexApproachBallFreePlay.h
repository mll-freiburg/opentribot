#ifndef _TRIBOTS_BCOMPLEXAPPROACHBALLFREEPLAY_H_
#define _TRIBOTS_BCOMPLEXAPPROACHBALLFREEPLAY_H_

#include "../../BDIBehavior.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{

/**
 * Fährt den Ball auf dem gesamten sinnvoll Spielfeld an. Mit den 
 * Standardeinstellungen wird der Ball ab kurz vor der Mittellinie und im 
 * Vorfeld direkt, im Bereich der Seitenlinien von Hinten in Richtung des 
 * gegnerischen Tores und vor dem eigenen Tor von Hinten in Richtung der 
 * Seitenlinie angefahren. 
 * Mit den optionalen Parametern können andere Bereichefür die direkte 
 * Ballanfahrt (directArea) und die Ballanfahrt von Hinten Richtung Seitenlinie 
 * (toOutsideArea) angegeben werden. In allen Bereichen, die nicht von diesen
 * beiden Areas erfasst werden, wird der Ball von Hinten in Richtung 
 * gegnerischem Tor angefahren. Zu den Eigenschaften der drei verschiedenen
 * Ballanfahrten siehe auch BApproachBallDirectly, 
 * BApproachBallFromBehindPointingToGoal und 
 * BApproachBallFromBehindPointingAwayOwnGoal.
 */
class BComplexApproachBallFreePlay : public Tribots::BDIBehavior
{
public:
  /**
   * Konstruiert eine komplexe Ballanfahrt aus mehreren Unterverhalten. Die
   * Auswahl der Anfahrten geschieht nach der Ballposition an Hand einer festen
   * Aufteilung des Spielfeldes in Aktionsbereiche.
   */
  BComplexApproachBallFreePlay() throw();
                               
  /** Destruktor */
  virtual ~BComplexApproachBallFreePlay() throw();
};

}

#endif //_BCOMPLEXAPPROACHBALLFREEPLAY_H_
