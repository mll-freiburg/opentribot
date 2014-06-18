#include "ImageClickWidget.h"
#include "MainWidget.h"
#include "MainWindow.h"
#include "../projectmanagement/ProjectManager.h"
#include "../datatypes/ModelPoints.h"

#include "../../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/Formation/Image.h"
#include "../../../../ImageProcessing/Formation/YUVImage.h"
#include "../../../../ImageProcessing/Formation/ImageIO.h"

#include <qtabwidget.h>
#include <qmainwindow.h>

#define U_DIM 128
#define V_DIM 128
#define SCALE 5

/**  */
ImageClickWidget::ImageClickWidget(QTabWidget* parent, const char* name, 
									WFlags f) : TribotsTools::ImageWidget(parent, name, f)
{
    
    //With mousetracking enabled, this method is called everytime
	//when the mouse is being moved. Otherwise the mouse would have to
	//be pressed as well.
	QWidget::setMouseTracking(true);
	parentWidget = parent;
	precision = false;
	zooming = false;
	clickCount = 0;
}

ImageClickWidget::ImageClickWidget(const Tribots::Image& image, QWidget* parent, 
		 WFlags f) : TribotsTools::ImageWidget(image, parent, f)
{
	
}


ImageClickWidget::~ImageClickWidget()
{
	
}


void ImageClickWidget::mouseMoveEvent (QMouseEvent *e)
{
	this->setFocus();
	
	if(zooming) {
		QPoint pos = e->pos();
	   	zoom(pos.x(), pos.y());
	   	int tabid = parentWidget->indexOf(this);
	   	setImage(*((ProjectManager::getProject(ProjectManager::getActProjectID())).getCalibImages()).at(tabid).getZoomImage());
	}
}

void ImageClickWidget::mousePressEvent (QMouseEvent *e)
{
	
	parentWidget->setFocus();
	
	// aktuellen Tab bestimmen
   	int tabid = parentWidget->indexOf(this);
   	
   	// ProjectID besorgen, hängt an Menüeintrag der offenen Projects
   	int proID = ProjectManager::getActProjectID();
   	
   	// Index des aktuellen Punktes besorgen
   	int index = getModelView()->getActPointIndex();
	
	clickCount++;
	
	if(clickCount > (((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).size()) {
   		clickCount = (((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).size();
   		return;	
   	}
	
	// schon gesetzter Punkt wird überschrieben
	if(((((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(index)).getClicked()) {
		clickCount--;	
	}
	
	// Maus Position bestimmen
   	QPoint pos = e->pos();
   	
   	if(precision){ //sub-pixel precision triggered by Ctrl-Key
  		//calc displacement in scaled distance and add it to the orignial point
  		((((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(index)).setXWorld((double)(((pos.x() - orig_x) / SCALE) + orig_x));
  		((((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(index)).setYWorld((double)(((pos.y() - orig_y) / SCALE) + orig_y));
  	} else { //normal click
  		// Position in WorldPointsVecor speichern
   		((((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(index)).setXWorld((double)pos.x());
   	   	((((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(index)).setYWorld((double)pos.y());
   	
  	}

//   	// testausgabe
//   	for(unsigned int i = 0; i < (((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).size(); i++) {
//   		std::cerr << (((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(i);
//   	}

   	markSelections();
   	   		
	// ModelPoints bearbeiten
	((((ProjectManager::getProject(proID)).getCalibImage(tabid)).getModelPoints()).at(index)).setClicked(true);

   	// ModelView aktualisieren
   	getModelView()->updateModelView(proID, tabid);
   	
   	// in ModelView einen Punkt weiter setzen
   	getModelView()->showNextPoint();
   	  
}

void ImageClickWidget::keyPressEvent ( QKeyEvent * e )
{

}

/**  */
void ImageClickWidget::keyReleaseEvent ( QKeyEvent * e )
{
	if (e->key() == Key_Space) { //Trigger moving within zoomed area
		
		if(precision) {
			precision = false;
		}
		else {
			precision = true;
		}
	}
	
	if(e->key() == Key_Control) {
		
		if(zooming) {
			zooming = false;
		}
		else {
			zooming = true;
		}
		
		int tabid = parentWidget->indexOf(this);
		setImage(*((ProjectManager::getProject(ProjectManager::getActProjectID())).getCalibImages()).at(tabid).getTempImage());	
	}
}

/**  */
int ImageClickWidget::getClickCount() { return clickCount; }

/**  */	
void ImageClickWidget::setClickCount(int count)
{
	clickCount = count;	
}


/**  */
ModelView* ImageClickWidget::getModelView()
{
	MainWidget* mainwidgettemp = dynamic_cast<MainWidget*>(parentWidget->parentWidget(true));
	MainWindow* mainwindowtemp = dynamic_cast<MainWindow*>(mainwidgettemp->parentWidget(true));
	return mainwindowtemp->modelview;
}

/**  */
void ImageClickWidget::markSelections() 
{
	// aktuellen Tab bestimmen
   	int tabid = parentWidget->indexOf(this);
   	
   	// ProjectID besorgen, hängt an Menüeintrag der offenen Projects
   	int proID = ProjectManager::getActProjectID();

	
	// richtiges Bild besorgen und Pixel setzen
   	((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).refreshTempImage();
   	Tribots::Image* image = ((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getTempImage();
   	Tribots::YUVTuple yuv;
   	yuv.y = 10;
   	yuv.u = 800;
   	yuv.v = 10;
   	QPoint pos;
   	
   	for(unsigned int j = 0; j < (((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getModelPoints()).size(); j++) {
   		
   		
   		pos.setX((int)((((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getModelPoints()).at(j)).getXWorld());
   		pos.setY((int)((((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getModelPoints()).at(j)).getYWorld());
   		
   		if(pos.x() != -1.0 && pos.y() != -1.0) {
		   	for(int i = 0; i < 5; i++) {
			   	
			   	image->setPixelYUV(pos.x(),pos.y(),yuv);
			   	image->setPixelYUV(pos.x()+i,pos.y(),yuv);
			   	image->setPixelYUV(pos.x()-i,pos.y(),yuv);
			   	image->setPixelYUV(pos.x(),pos.y()+i,yuv);
			   	image->setPixelYUV(pos.x(),pos.y()-i,yuv);
			   	
			   	image->setPixelYUV(pos.x()+1,pos.y()+1,yuv);
			   	image->setPixelYUV(pos.x()+i,pos.y()+1,yuv);
			   	image->setPixelYUV(pos.x()-i,pos.y()+1,yuv);
			   	image->setPixelYUV(pos.x()+1,pos.y()+i,yuv);
			   	image->setPixelYUV(pos.x()+1,pos.y()-i,yuv);
			   	
			   	image->setPixelYUV(pos.x()-1,pos.y()-1,yuv);
			   	image->setPixelYUV(pos.x()+i,pos.y()-1,yuv);
			   	image->setPixelYUV(pos.x()-i,pos.y()-1,yuv);
			   	image->setPixelYUV(pos.x()-1,pos.y()+i,yuv);
			   	image->setPixelYUV(pos.x()-1,pos.y()-i,yuv);
			   	
		   	}
   		}
   	}
   	setImage(*image);
}

/**  */
void ImageClickWidget::zoom(int u, int v){
  
  int tabid = parentWidget->indexOf(this);
  int proID = ProjectManager::getActProjectID();
  //Tribots::Image* tempImage = ((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getTempImage();
  Tribots::Image* tempImage = ((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getTempImage();
  Tribots::Image* zoomImage = ((ProjectManager::getProject(proID)).getCalibImages()).at(tabid).getZoomImage();
  
  if (tempImage == NULL || zoomImage == NULL) return;
  if (u >= zoomImage->getWidth()) u = zoomImage->getWidth()-1;
  if (v >= zoomImage->getHeight()) v = zoomImage->getHeight()-1;
  if (u < 0) u = 0;
  if (v < 0) v = 0;
  if (precision) return; //Move mouse in zoomed area --> Therefore, do not change zoom
  //else zoom area underneath mouse position
  
  //remember current original location.
  orig_x = u;
  orig_y = v;
  
  Tribots::ImageBuffer::convert(tempImage->getImageBuffer(), 
		       zoomImage->getImageBuffer());

  //draw crosses to the already selected points
  //markSelections();
  
  Tribots::YUVTuple yuv;

  int i = u - U_DIM/2;
  int j = v - V_DIM/2;
  i = i < 0 ? 0 : i;
  j = j < 0 ? 0 : j;
  int org_j = j;
  int i_to = (u + U_DIM/2) > zoomImage->getWidth() ? zoomImage->getWidth() : u + U_DIM/2;
  int j_to = (v + V_DIM/2) > zoomImage->getHeight() ? zoomImage->getHeight() : v + V_DIM/2;

  int di, dj;
  for (; i < i_to; ++i){
  	for (; j<j_to; ++j){
	        di = i - u;
		dj = j - v;
		tempImage->getPixelYUV (u+di/SCALE, v+dj/SCALE, &yuv);
		zoomImage->setPixelYUV(i,j, yuv);
	}
	j = org_j;
  }
 //end zoom underneath mouse
}