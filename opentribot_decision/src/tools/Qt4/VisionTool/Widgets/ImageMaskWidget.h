
#ifndef _TribotsTools_ImageMaskWidget_h_
#define _TribotsTools_ImageMaskWidget_h_

#include "UI/ImageMaskWidget.h"
#include "VisionToolWidget.h"
#include "../Logic/VisionToolImageSource.h"
#include "../../../../ImageProcessing/PixelAnalysis/RobotMask.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/PixelAnalysis/ColorClassifier.h"
#include "../../../../ImageProcessing/ObjectAnalysis/ChainCoding.h" 
#include "../../components/MorphologicOperators.h"
 
namespace TribotsTools {

  class ImageMaskWidget : public VisionToolWidget, private Ui::ImageMaskWidget {
    Q_OBJECT

  public:
    ImageMaskWidget (VisionToolImageSource& , Tribots::ConfigReader&, QStatusBar&, QWidget* =0, Qt::WindowFlags =0);
    ~ImageMaskWidget ();

  private slots:
		void recordToggled(bool);
    void showmaskToggled(bool);
		void autoresetClicked();
		void reloadClicked();
    void mousePressedInImage(QMouseEvent*);
    void mouseMovedInImage(QMouseEvent*);
    void keyPressEvent(QKeyEvent*);
		void setOpacity(int);
		void setBrush(int);
		void setThresh(int);

  public:
		void fillRegion(Tribots::Image * image, const Tribots::Region &region, const Tribots::RGBTuple &color);
		bool loadMask();
		void setRobotMaskCursor(int size);
    void start();
    void stop();
    void loop();


  private:
		int diffmax;
		bool debug;
		bool fillholes;
	
    bool started;
    bool autom;
    bool showmask;
		bool record;
		unsigned int threshold;
		unsigned int opacity;
		unsigned int brush;
		int recentMousePressX;
    int recentMousePressY;
		Qt::MouseButton button;

		Tribots::RobotMask* imageMask;

    VisionToolImageSource& imageSource;
    Tribots::Image* oldimage;
    Tribots::Image* diffimage;
 	
	  };

}

#endif
