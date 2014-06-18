
#include "ImageMaskWidget.h"
#include <cmath>
#include "../../../../ImageProcessing/Formation/Painter.h"

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) <= (b) ? (a) : (b))

#include <QtGui/QBitmap>
 
using namespace TribotsTools;
using namespace Tribots;
using namespace std;

ImageMaskWidget::ImageMaskWidget(VisionToolImageSource& is, Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* p, Qt::WindowFlags f) : VisionToolWidget (cfg,stb,p,f), debug(false), fillholes(true), started (false), record(0), threshold(80), brush(2), imageMask(NULL), imageSource(is), oldimage(NULL), diffimage(NULL) {
  setupUi (this);
  connect (checkBox_show, SIGNAL(toggled(bool)), this, SLOT(showmaskToggled(bool)));
  connect (pushButton_record, SIGNAL(toggled(bool)), this, SLOT(recordToggled(bool)));
  connect (pushButton_autoreset, SIGNAL(clicked()), this, SLOT(autoresetClicked()));
  connect (pushButton_reload, SIGNAL(clicked()), this, SLOT(reloadClicked()));
  connect (imageWidget, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressedInImage(QMouseEvent*)));
  connect (imageWidget, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseMovedInImage(QMouseEvent*)));
  connect (imageWidget, SIGNAL(keyPressed(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));
	connect (horizontalSlider_opacity, SIGNAL(valueChanged(int)), this, SLOT(setOpacity(int)));
	connect (horizontalSlider_brushsize, SIGNAL(valueChanged(int)), this, SLOT(setBrush(int)));
	connect (horizontalSlider_threshold, SIGNAL(valueChanged(int)), this, SLOT(setThresh(int)));
  setFocusPolicy (Qt::StrongFocus);
}

ImageMaskWidget::~ImageMaskWidget () {
  if (started)
    stop();
}

// slot to start and stop recording of the image mask
void ImageMaskWidget::recordToggled(bool b) {
  record=b;
	if (record) {
		pushButton_autoreset->setEnabled(false);
		horizontalSlider_threshold->setEnabled(false);
		label_threshold->setEnabled(false);
		pushButton_reload->setEnabled(false);
	} else {
		pushButton_autoreset->setEnabled(true);
		horizontalSlider_threshold->setEnabled(true);
		label_threshold->setEnabled(true);
		pushButton_reload->setEnabled(true);

		//refresh view
		setThresh(threshold);
	}
}

void ImageMaskWidget::showmaskToggled(bool b) {
  showmask=b;
}

void ImageMaskWidget::setOpacity(int value) {
  opacity = value;
}

// slot to set the brush size
void ImageMaskWidget::setBrush(int value) {
	setRobotMaskCursor(value);
	brush=value;
}

// thresholds the higher depth difference image to extract a image mask bitmap
void ImageMaskWidget::setThresh(int value) {
	threshold=value;

	Tribots::Image* mask = new Tribots::RGBImage(diffimage->getWidth(), diffimage->getHeight());
	RGBTuple white = {255,255,255};
	RGBTuple black = {0,0,0};

	YUVTuple yuv;

	if (diffimage) {
		for (int i=0; i< diffimage->getWidth(); i++) {
			for (int j=0; j< diffimage->getHeight(); j++) {
				diffimage->getPixelYUV(i, j, &yuv);
	
				if (yuv.y > threshold) {
			//		imageMask->set(i, j, true);
					mask->setPixelRGB(i,j, white);
				} else {
			//		imageMask->set(i, j, false);
					mask->setPixelRGB(i,j, black);
				}
			}
		}

	//remove holes from thresholded image
  WhiteClassifier * whiteClassifier = new WhiteClassifier();
  BlackClassifier * blackClassifier = new BlackClassifier();
  RegionDetector * regionDetector = new RegionDetector();

  int whiteRegionMin = 1000;
  int blackRegionMin = 1000;

	if (fillholes) {
		
		try {
			
			// Erosionsoperator anwenden um kleine weisse unterbrechungen zu entfernen
			// und die Maske (schwarze bereiche) an den Objektraendern um ein Pixel zu
			// verbreitern
			Erosion erosion = Erosion(3);
			Image* tmp = erosion(*mask);
			delete mask;
			mask = tmp;
			
			// Rand schwarerzen
			mask->setBlackBorder();
			
			mask->setClassifier(whiteClassifier);
			vector<Tribots::Region> regions;
			regionDetector->findRegions(*mask, 1, &regions);
			
			for (unsigned int i=0; i < regions.size(); i++) {
				if (regions[i].getArea() < whiteRegionMin) {
					fillRegion(mask, regions[i], black);
				}
			}
			
			// Rand weissen  
			mask->setWhiteBorder();
			
			// kleine schwarze Flecken mit weiss fuellen
			mask->setClassifier(blackClassifier);
			regions.clear();
			regionDetector->findRegions(*mask, 1, &regions);
			
			for (unsigned int i=0; i < regions.size(); i++) {
				if (regions[i].getArea() < blackRegionMin) {
					fillRegion(mask, regions[i], white);
				}
			}
			
			for (int i=0; i < 2; i++) {
				tmp = erosion(*mask);
				delete mask;
				mask = tmp;
			}
		} catch(TribotsException& e){
			cerr << e.what() << '\n';
			
		}
		
	}
	
		for (int i=0; i< diffimage->getWidth(); i++) {
			for (int j=0; j< diffimage->getHeight(); j++) {
				mask->getPixelYUV(i, j, &yuv);
				
				if (yuv.y > 100) {
					imageMask->set(i, j, true);
				} else {
					imageMask->set(i, j, false);
				}
			}
		}

	}
}

/* Achtung: Loecher werden NICHT gefllt!!!!  Dazu muss diese Funktion auch noch
   mit der Kante des Lochs aufgerufen werden!  Das stellt allerdings ein Problem dar,
   weil erstmal das loch gesucht werden muss... 
   
   Zur Zeit werden Loecher daher einfach nicht richtig behandelt ! */
void ImageMaskWidget::fillRegion( Tribots::Image * image, 
                                      const Tribots::Region & region,
                                      const Tribots::RGBTuple & color )
{
  int x = region.x;
  int y = region.y;
  
  for (unsigned int i=0; i < region.chainCode.size(); i++) {
    int dir = region.chainCode[i];
    
    if (dir == 3 || dir == 0) {    // bei jedem Schritt nach unten oder nach rechts
      int xImg = x;
      while (image->getPixelClass(xImg,y) == region.colClass && 
                xImg < image->getWidth()) {
        image->setPixelRGB(xImg, y, color);
        xImg++;
      }
    } 
    x += ChainCoder::xDir[dir];
    y += ChainCoder::yDir[dir];
  } 
  
  // sonderbehandlung fr erstes==letztes 
  // pixel  und den fall, dass die Fl�he nur ein pixel hat
  int xImg = x;
  while (image->getPixelClass(xImg,y) == region.colClass && 
         xImg < image->getWidth()) {
    image->setPixelRGB(xImg, y, color);
    xImg++;
  } 
}


// slot to reset image mask and difference image to black
void ImageMaskWidget::autoresetClicked() {

	// reset image mask
	if (imageMask) delete imageMask;
	imageMask = new RobotMask (diffimage->getWidth(), diffimage->getHeight(), false);
	
	// reset difference map
	if (diffimage) {
		RGBTuple black = {0, 0, 0};
		for (int i=0; i< diffimage->getWidth(); i++) {
			for (int j=0; j< diffimage->getHeight(); j++) {
				diffimage->setPixelRGB(i, j, black);
			}
		}
	}
	
	checkBox_show->setChecked(true);	
	horizontalSlider_threshold->setEnabled(false);
	label_threshold->setEnabled(false);
	pushButton_autoreset->setEnabled(false);
}

// slot to reset image mask to previously saved version
void ImageMaskWidget::reloadClicked() {
	loadMask();
	checkBox_show->setChecked(true);	
}

void ImageMaskWidget::mousePressedInImage(QMouseEvent* event) {
	if (showmask) {
		recentMousePressX = event->x();
		recentMousePressY = event->y();
		button = event->button();
			
		// draw one point
		mouseMovedInImage(event);
	}
}

void ImageMaskWidget::mouseMovedInImage(QMouseEvent* event) {
	if (showmask) {
		int currentMousePressX = event->x();
		int currentMousePressY = event->y();
		
		int numX = abs (currentMousePressX - recentMousePressX);
		int numY = abs (currentMousePressY - recentMousePressY);
		int num = (numX<numY ? numY : numX) +1;
		
		// several brushes in diferent sizes
		int brushsizes[4] = {4, 6, 10, 18};
		int brushstart[4][18]=
			{{-0, -1, -1, -0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
			 {-0, -1, -2, -2, -1, -0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
			 {-1, -2, -3, -4, -4, -4, -4, -3, -2, -1,  0,  0,  0,  0,  0,  0,  0,  0},
			 {-2, -4, -5, -6, -7, -7, -8, -8, -8, -8, -8, -8, -7, -7, -6, -5, -4, -2}};
		int brushend[4][18] =
			{{1, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			 {1, 2, 3, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			 {2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 0, 0, 0, 0, 0, 0, 0, 0},
			 {3, 5, 6, 7, 8, 8, 9, 9, 9, 9, 9, 9, 8, 8, 7, 6, 5, 3}};
		
		int zeilen = brushsizes[brush];
		
		// draw line of brush tips, bresenham style
		for (int k=0; k<=num; k++) {
			double theta = (double)k / (double)num;
			int x=(int)(theta*recentMousePressX+(1-theta)*currentMousePressX);
			int y=(int)(theta*recentMousePressY+(1-theta)*currentMousePressY);
					
			for (int i=0; i<zeilen; i++) {
				for (int j=brushstart[brush][i]; j<=brushend[brush][i]; j++) {
					imageMask->set(x+i-zeilen/2,y+j-1, button == Qt::RightButton);
				}
			}
		}
		recentMousePressX = currentMousePressX;
		recentMousePressY = currentMousePressY;
		
		imageSource.notify ("mask");
	}
}

void ImageMaskWidget::keyPressEvent(QKeyEvent* event) {
  if (showmask) {
    switch (event->key()) {
      case Qt::Key_Up:	break;
      case Qt::Key_Down: break;
      case Qt::Key_Right: break;
      case Qt::Key_Left: break;
      case Qt::Key_Plus: break;
      case Qt::Key_Minus: break;
      case Qt::Key_F: 
				fillholes=!fillholes;
				ImageMaskWidget::setThresh(threshold);
				break;
      case Qt::Key_D: 
				debug=!debug;
				break;
      default: break;
    }
  }
}

// loads several brush tips from images
void ImageMaskWidget::setRobotMaskCursor(int size) {
	QBitmap map, mask;

	if (size >= 0) {
		switch(size) {
			case 0:
				map.load("images/map02.png");
				mask.load("images/mask02.png");
				break;
			case 1:
				map.load ("images/map04.png");
				mask.load ("images/mask04.png");
				break;			
			case 2: 
				map.load ("images/map08.png");
				mask.load ("images/mask08.png");
				break;
			case 3: 
				map.load ("images/map16.png");
				mask.load ("images/mask16.png");
				break;
			default:
				map.load ("images/map04.png");
				mask.load ("images/mask04.png");
		}
		QCursor ballcursor(map, mask);
		imageWidget->setCursor (ballcursor);
	} else {
		imageWidget->setCursor (Qt::ArrowCursor);
	}
}

// read image mask configuration and load the mask if possible 
bool ImageMaskWidget::loadMask() {
  Tribots::Image& image (imageSource.getImage());

  string imageMaskFile ="";
  string section;
  config.get("VisionTool::Section", section);
  config.get( (section + "::robot_mask_file").c_str(), imageMaskFile);
	
	if (imageMask) {
		delete imageMask;
	}
	
	if (imageMaskFile.length() > 0) {
		try {
      imageMask = new RobotMask (imageMaskFile.c_str());

			statusBar.showMessage(("Bildmaske: " + imageMaskFile).c_str());
		} catch(TribotsException& e){
			cerr << e.what() << '\n';
		
			//new mask, all filled
			imageMask = new RobotMask (image.getWidth(), image.getHeight(), true);

			pushButton_reload->setEnabled(false);
			statusBar.showMessage("Die Bildmaske konte nicht geladen werden! Es wurde eine leere Maske erstellt.");
			return false;
		}
  } else {
		imageMask = new RobotMask (image.getWidth(), image.getHeight(), true);
		pushButton_reload->setEnabled(false);
		statusBar.showMessage(("Die Sektion " + section + " der Konfigurationsdatei enthaelt keinen Eintrag ueber die Bildmaske (robot_mask_file)").c_str());
		return false;
	}
	
	return true;
}

void ImageMaskWidget::start () {
  started=true;

  imageSource.setMode (true, false);
  imageWidget->centerImage();

	// if tab starts first time, deactivate image reset
	if(!diffimage) {
		horizontalSlider_threshold->setEnabled(false);
		label_threshold->setEnabled(false);
		pushButton_autoreset->setEnabled(false);
	}

	// only load the mask if needed, not on comeback
	if (!imageMask) {
		loadMask();
		// start with small brush
		setRobotMaskCursor(2);

		checkBox_show->setChecked(false);
		checkBox_show->setChecked(true);
		horizontalSlider_opacity->setSliderPosition(50);
	} else {
		// at least show it
		checkBox_show->setChecked(true);
	}
}

void ImageMaskWidget::stop () {
  started=false;
  imageSource.setMode ();  // default-Modus, tut nichts
}

void ImageMaskWidget::loop () {
  Tribots::Image& image (imageSource.getImage());

	//TODO too slow
	Tribots::Image * outimage = image.clone();

	//to prevent crash on first image
	if(!oldimage) {
		oldimage = image.clone();
	}
	
	YUVTuple yuv1;
	YUVTuple yuv2;
	
	// if recording starts, create a new difference image if necessary and clear it
	if (record) {
		if (!diffimage) {

			//TODO: unhack
			RGBTuple black = {0, 0, 0};
			diffimage = image.clone();
			for (int i=0; i< image.getWidth(); i++) {
				for (int j=0; j< image.getHeight(); j++) {
					diffimage->setPixelRGB(i, j, black);
				}
			}
		}
		
		
		
		// calculate differences between actual and last image
		for (int i=0; i< image.getWidth(); i++) {
			for (int j=0; j< image.getHeight(); j++) {
				image.getPixelYUV(i,j,&yuv1);
				oldimage->getPixelYUV(i,j,&yuv2);
				
				
//				int diff= abs(yuv1.u - yuv2.u) + abs(yuv1.v - yuv2.v);
				int diff= min(255, abs(yuv1.u - yuv2.u) + abs(yuv1.v - yuv2.v) + abs(yuv1.y - yuv2.y)/2);
				
				
				YUVTuple diffpix;
				diffimage->getPixelYUV(i, j, &diffpix);
				if (diffpix.y < diff) {
					diffpix.y = diff;
				}

				// save the difference
				diffimage->setPixelYUV(i,j, diffpix);
				
				//for the user to see something
				YUVTuple vis;
				if (diff > 10) { 
						vis.y = min (3 * diff + diffpix.y, 255);
				} else {
						vis.y = diffpix.y;
						vis.u=128;
						vis.v=128;
				}
				outimage->setPixelYUV(i, j, vis);
			}
		}
	} else {
		// not recording
		
		if (showmask) {
			RGBTuple rgb;
			
			int transparency = 100 - opacity;
			
			for (int i=0; i< image.getWidth(); i++) {
				for (int j=0; j< image.getHeight(); j++) {
					if (!imageMask->isValid(i,j)) {
						image.getPixelRGB(i,j,&rgb);
						
						// mask overlay with transparency
						rgb.r = (max((int)(rgb.r + (255 - rgb.r) /2), 0));
						rgb.g = (max((int)(rgb.g * transparency / 100.0), 0));
						rgb.b = (max((int)(rgb.b * transparency / 100.0), 0));
						
						outimage->setPixelRGB(i,j,rgb);
					}
				}
			}
		}
	}	
		
	if (debug) {
		imageWidget->setImage (*diffimage);
	} else {
		imageWidget->setImage (*outimage);
	}

	if(outimage) {
		delete outimage;
	}
	if(oldimage) {
		delete oldimage;
	}
	oldimage = image.clone();

}
