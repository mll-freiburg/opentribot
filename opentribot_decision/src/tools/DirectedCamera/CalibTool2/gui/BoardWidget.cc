#include <iostream>
#include <vector>

#include <qpushbutton.h>
#include <qevent.h>
#include <qframe.h>
#include <qlayout.h>
#include <qpixmap.h>
#include <qpainter.h>
#include "BoardWidget.h"
#include "../projectmanagement/ProjectManager.h"

BoardWidget::BoardWidget(QWidget *parent, const char *name)
    : QFrame(parent)
{
    setFrameStyle(QFrame::Sunken | QFrame::StyledPanel);
}

BoardWidget::~BoardWidget() 
{
	
}

void BoardWidget::clear()
{
    m_pixmap.fill(Qt::white);
    update();
}

void BoardWidget::drawModelPoints(int projectID, int picIndex, int actPointIndex)
{
	std::vector<ModelPoints> modelPoints = ((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints();
	
	// zum anfang einmal alles loeschen
	clear();
	
	// Abstand vom Rand festlegen
	double abstand = 20.0;
	
	// QPainter anlegen
	QPainter painter(&m_pixmap);
	
	// groessten x-wert und groessten y-wert aus ModelPoints
	double xmax = 0;
	double ymax = 0;
	for(unsigned int i = 0; i < modelPoints.size(); i++) {
		if((modelPoints.at(i)).getXModel() > xmax) {
			xmax = (modelPoints.at(i)).getXModel();
		}
		if((modelPoints.at(i)).getYModel() > ymax) {
			ymax = (modelPoints.at(i)).getYModel();
		}
	}
	
	// breite des Koordinaten Systems
	double width = m_pixmap.width()-(abstand*2);
	// hoehe des Koordinaten Systems
	double height = m_pixmap.height()-(abstand*2);
	
	// Koordinaten System zeichnen
	// x-Richtung
	for(int i = abstand; i <= m_pixmap.width()-abstand; i++) {
		painter.fillRect(i, m_pixmap.height()-abstand, 1, 1, Qt::black);	
	}
	// y-Richtung
	for(int i = abstand; i <= m_pixmap.height()-abstand; i++) {
		painter.fillRect(abstand, i, 1, 1, Qt::black);
	}
	
	// Pfeile zeichnen y-Richtung
	int j = abstand-1;
	int k = abstand+1;
	for(int i = abstand+1; i <= abstand+2; i++) {
		painter.fillRect(j--, i, 1, 2, Qt::black);
		painter.fillRect(k++, i, 1, 2, Qt::black);	
	}	
	// Pfeile zeichnen x-Richtung
	int l = m_pixmap.height()-abstand-1;
	int m = m_pixmap.height()-abstand+1;
	for(int i = m_pixmap.width()-(abstand+2); i >= m_pixmap.width()-(abstand+3); i--) {
		painter.fillRect(i, l--, 2, 1, Qt::black);
		painter.fillRect(i, m++, 2, 1, Qt::black);	
	}
	// Koordinatenachsen bezeichnen
	painter.drawText ( abstand+10, abstand, "ymax", -1, QPainter::Auto );
	painter.drawText ( m_pixmap.width()-(abstand+30), m_pixmap.height()-5, "xmax", -1, QPainter::Auto );
	
	// Punkte, die noch nicht behandelt worden -> gruen
	// Punkte, die schon behandelt worden -> rot
	// Punkt, der aktuell ausgewählt ist ->rot umrandet
	for(unsigned int i = 0; i < modelPoints.size(); i++) {
		
		double x = (modelPoints.at(i)).getXModel();
		double y = (modelPoints.at(i)).getYModel();
				
		if((modelPoints.at(i)).getClicked()) {
			// setze gruenen Punkt
			painter.fillRect(((width/xmax)*x)+abstand-2, (height-((height/ymax)*y))+abstand-2, 5, 5, Qt::red);	
		
		} else if(!(modelPoints.at(i)).getClicked()) {
			// setze schwarzen Punkt
			painter.fillRect(((width/xmax)*x)+abstand-2, (height-((height/ymax)*y))+abstand-2, 5, 5, Qt::green);	    		
		}
		
		if(i == actPointIndex) {
    		painter.setPen(Qt::red);
    		painter.drawRect(((width/xmax)*x)+abstand-4, (height-((height/ymax)*y))+abstand-4, 9, 9);
    	}
    	
	}
	update();
	
}

void BoardWidget::resizeEvent(QResizeEvent *event)
{
    QPixmap new_pixmap(event->size());
    new_pixmap.fill(Qt::white);
    QPainter painter(&new_pixmap);
    painter.drawPixmap(QPoint(0, 0), m_pixmap);
    m_pixmap = new_pixmap;
}

void BoardWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.drawPixmap(QPoint(0, 0), m_pixmap);
    QFrame::paintEvent(event);
}
