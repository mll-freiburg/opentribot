#ifndef BOARDWIDGET_H_
#define BOARDWIDGET_H_

#include <vector>

#include <qframe.h>
#include <qpixmap.h>
#include "../datatypes/ModelPoints.h"
#include "../projectmanagement/ProjectManager.h"

class BoardWidget : public QFrame
{
    Q_OBJECT
    
	public:
	    BoardWidget(QWidget *parent = 0, const char *name = 0);
	    ~BoardWidget();
	
		void drawModelPoints(int projectID, int picIndex, int index);
	    void clear();    
	
	protected:
	    void resizeEvent(QResizeEvent *event);
	    void paintEvent(QPaintEvent *event);
	
	private:
	    QPixmap m_pixmap;
	    
};

#endif /*BOARDWIDGET_H_*/
