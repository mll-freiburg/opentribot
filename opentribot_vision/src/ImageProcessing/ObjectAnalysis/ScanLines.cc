
#include "ScanLines.h"


namespace Tribots {

  using namespace std;

  // Linien an der Maske und am Bildrand abschneiden bzw. aufteilen. Wurde eine
  // RobotMask geladen, wird diese verwendet, um sowohl nicht maskierte Pixel
  // als auch Pixel ausserhalb des Bildbereiches zu finden (isValid macht 
  // beides). Wurde keine Maske geladen, wird die Position auf der Scanlinie
  // nur daraufhin ueberprueft, ob sie im Bildbereich liegt.

  void ScanLines::insertScanLine(const RobotMask* robotMask, const LineSegment& line, 
				 int width, int height)
  {
    Vec direction = line.p2 - line.p1;
    int maxSteps = (int) direction.length();
    int steps = 0;

    direction = direction.normalize();

    Vec pos=line.p1;           // Startpunkt
    for (; steps <= maxSteps; steps++) {// Ersten gueltigen Punkt suchen
      if ((robotMask != 0 && robotMask->isValid((int)pos.x, (int)pos.y)) ||
	  (robotMask == 0 && pos.x >= 1 && pos.y >= 1 && 
	   pos.x+1 < width && pos.y+1 < height)) {
	break;                 // Punkt ist gueltig
      }
      pos = pos + direction;
    }
    if (steps >= maxSteps) {   // Linie maximal ein Punkt groß -> uninteressant
      return;
    }
    Vec startPos = pos;        // Anfang merken
    for (; steps <= maxSteps; steps++) {// vorangehen, bis Punkt ungueltig
      bool isInvalidRobotMask = (robotMask != 0 && ! robotMask->isValid((int)pos.x, (int)pos.y));
      bool isCloseBorder = (pos.x < 2 || pos.y < 2 || pos.x+2 >= width || pos.y+2 >= height);
      if (isInvalidRobotMask || isCloseBorder) {
        scanLines.push_back(LineSegment(startPos, pos - direction));
	if (isInvalidRobotMask && !isCloseBorder) {
	  insertScanLine(robotMask, LineSegment(pos+direction, line.p2), width, height); 
	}
	return ;               // Fertig!
      }
      pos = pos + direction;
    }
    scanLines.push_back(LineSegment(startPos, line.p2)); // Linie bis Ende ok
  }

  ScanLines::ScanLines(const RobotMask* robotMask, const Vec& middle, int innerRadius, int outerRadius, 
		       int width, int height, int n, bool kaestchen) 
  {    

    if (kaestchen) {
      int distance = (int)(innerRadius * 1.4);
      for (int i=0; i < 9; i++) {
        LineSegment horizontal(Vec(-distance,0), Vec(distance, 0));
        LineSegment vertical(Vec(0,-distance), Vec(0, distance));
        insertScanLine(robotMask, horizontal.translate(Vec(0,-distance)).translate(middle), 
                       width, height);
        insertScanLine(robotMask, horizontal.translate(Vec(0, distance)).translate(middle), 
                       width, height);
        insertScanLine(robotMask, vertical.translate(Vec(-distance, 0)).translate(middle), 
                       width, height);
        insertScanLine(robotMask, vertical.translate(Vec( distance, 0)).translate(middle), 
                       width, height);
        distance+= 13;
      }
    }



    Angle stepAngle = Angle::deg_angle(360. / n); // Schrittweite berechnen
    Angle halfStepAngle = stepAngle;             // Schrittweite für die kurze
    halfStepAngle*=0.5;                          // Zwischenlinie

    LineSegment baseLine1(Vec(0, innerRadius), Vec(0, outerRadius));
    LineSegment baseLine2(Vec(0, (2*innerRadius+outerRadius)/3),
			  Vec(0, outerRadius));
    baseLine2.s_rotate(halfStepAngle);

    // Radiale Scanlinien:
    // Vorgehensweise (i*Mal):
    // 1. Standardlinie von (0,innerRadius) nach (0,outerRadius) und zweite
    //    Line von (0, (innerRadius+outerRadius)/2) nach (o, outerRadius),
    //    die um stepAnlge/2 Grad rotiert wurde (Verdichtung in den Aussen-
    //    bereichen des Bildes)
    // 2. Rotation um stepAngle * i (Schrittweite)
    // 3. Translation zum Mittlepunkt middle

    // hierarchisches einfuegen der langen Scanlinien
    vector<bool> line_set (n);
    for (int i=0; i<n; i++)
      line_set[i]=false;
    int num_lines_set=0;
    
    // 0. Linie einfuegen
    insertScanLine (robotMask, baseLine1.translate (middle),width,height);
    line_set[0]=true;
    num_lines_set++;

    // weitere Linien hierarchisch einfuegen
    int base=2;
    while (num_lines_set<n) {
      for (int i=1; i<base; i+=2) {
	int j = (n*i)/base;
	if (!line_set[j]) {
	  insertScanLine (robotMask, baseLine1.rotate (j*stepAngle).translate (middle),width,height);
	  line_set[j]=true;
	  num_lines_set++;
	}
      }
      base*=2;
    }
    
    // hierarchisches einfuegen der kurzen Scanlinien
    // 0. Linie einfuegen
    insertScanLine (robotMask, baseLine2.translate (middle),width,height);
    num_lines_set--;
    line_set[0]=false;

    // weitere Linien hierarchisch einfuegen
    base=2;
    while (num_lines_set>0) {
      for (int i=1; i<base; i+=2) {
	int j = (n*i)/base;
	if (line_set[j]) {
	  insertScanLine (robotMask, baseLine2.rotate (j*stepAngle).translate (middle),width,height);
	  line_set[j]=false;
	  num_lines_set--;
	}
      }
      base*=2;
    }

  }
  
  ScanLines::ScanLines() {}
  
  ScanLines* 
  ScanLines::createForDirectedCamera(const RobotMask* robotMask, 
                                     int imageWidth, int imageHeight, 
                                     int nScanLines)
  {
    ScanLines* sl = new ScanLines();
    double stepWidth= imageWidth / (nScanLines / 2.);
    for (int i=0; i < nScanLines / 2; i++) {
      LineSegment line1(Vec(stepWidth*i+stepWidth/2, imageHeight-5), 
                        Vec(stepWidth*i+stepWidth/2+imageWidth/2, 10));  // Abstand von oben einfuegen?
      LineSegment line2(Vec(imageWidth-(stepWidth*i+stepWidth/2), imageHeight-5), 
                        Vec(imageWidth-(stepWidth*i+stepWidth/2+imageWidth/2), 10));
      sl->insertScanLine(robotMask, line1, imageWidth, imageHeight);
      sl->insertScanLine(robotMask, line2, imageWidth, imageHeight);
    }
    return sl;
  }

  ScanLines::ScanLines(const RobotMask* robotMask, const Vec& start, const Vec& end, int width, int height)
  {
    insertScanLine(robotMask, LineSegment(start, end), 
		   width <= 0 ? 999999 : width, 
		   height <= 0 ? 999999 : height);
  }
}
