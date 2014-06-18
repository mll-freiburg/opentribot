#ifndef _goaldetector_h_
#define _goaldetector_h_

#include "../Formation/Image.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../../Structures/VisibleObject.h"

namespace Tribots {

  class GoalDetector {
  public:
    /**
     * Erzeugt einen Tordektektor, der in einem Fenster der angegebenen
     * Höhe und Breite n zufällig ausgewählte Pixel auf ihre Farbklasse
     * untersucht.
     */
    GoalDetector(int centerX, int centerY, 
		 const WorldImageMapping* mapping = 0,
		 int width = 65, int height = 65, int n = 80);
    virtual ~GoalDetector() { ; }

    /**
     * Benutzt das Weltmodell, um die Position der Tore abzuschätzen und
     * sucht dort in einem Suchfenster nach gelben und blauen Pixeln.
     * Werden von der einen Farbe genügend viele und von der anderen
     * sehr wenige gesehen, gilt ein Tor als gefunden.
     */
    virtual void searchGoals(const Image&, Time time,
			     VisibleObjectList* vol)
      throw (TribotsException);

    void setVisualize(Image* vis) { this->vis=vis; }

  protected:
    /** Checks the area around the given image position for the dominant
     *  color (blue, yellow or "no dominant color"). Uses some fixed 
     *  thresholds. */
    int getDominantColor(const Image& image, const Vec& position, int n);
    /** Used to draw a bold rectangle of the color of the detected goal in the 
     *  image (vis) */
    void visualizeGoal(const Vec& pos, int col);

    Vec center;
    int width; 
    int height;
    int n;

    const WorldImageMapping* mapping;

    Image* vis;        ///< points to the user-requested debug output image
  };

};


#endif
