#ifndef IMAGECLICKWIDGET_H
#define IMAGECLICKWIDGET_H

#include "../../../../tools/components/ImageWidget.h"
#include "ModelView.h"
#include <qvariant.h>
#include <qwidget.h>
#include <qtabwidget.h>
#include <qmainwindow.h>

class ImageClickWidget : public TribotsTools::ImageWidget
{
	
	Q_OBJECT
	
	public :
		ImageClickWidget( QTabWidget* parent = 0, const char* name = 0, WFlags fl = 0 );
		ImageClickWidget(const Tribots::Image& image, QWidget* parent = 0, 
			 WFlags f = WType_TopLevel);
		~ImageClickWidget();
		
		void zoom(int u, int v);
		ModelView* getModelView();
		void markSelections();
		int getClickCount();
		void setClickCount(int count);
		
	public slots:
		void mouseMoveEvent (QMouseEvent *e);
		void mousePressEvent (QMouseEvent *e);
		void keyPressEvent ( QKeyEvent * e );
		void keyReleaseEvent ( QKeyEvent * e );
		
		
	private :
		QTabWidget* parentWidget; 
	
		bool precision;
		bool zooming;
		int clickCount;
		double orig_x;
		double orig_y;
		
}; // end ImageClickWidget

#endif // IMAGECLICKWIDGET_H
