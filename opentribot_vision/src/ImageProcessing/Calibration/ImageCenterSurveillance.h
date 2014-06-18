
#ifndef _Tribots_ImageCenterSurveillance_h_
#define _Tribots_ImageCenterSurveillance_h_

#include "../Formation/Image.h"
#include "../../Structures/GameState.h"

namespace Tribots {

  /** Klasse, die die Lage des Bildmittelpunktes ueberprueft und ggf. ins Journal und nach LOUT eine Warnung schreibt */
  class ImageCenterSurveillance {
  private:
    double cx, cy;
    Time timestampLatestGameStopped;
    bool wasStopped;
    bool checkDone;
  public:
    ImageCenterSurveillance (double centerX, double centerY);
    void setCenter (double centerX, double centerY);

    /** ueberpruefe Bildmittelpunkt anhand Bild image bei gegenwaertigem Refereestate state.
        Die Pruefung wird nur bei stopRobot durchgefuehrt und pro stopRobot-Episode nur ein
        Mal 2 Sekunden nach Beginn der Episode */
    void check (const Image& image, GameState state) throw ();
  };

}

#endif
