#ifndef MODELVIEW_H_
#define MODELVIEW_H_

#include <vector>

#include <qpushbutton.h>
#include <qevent.h>
#include <qframe.h>
#include <qlayout.h>
#include <qpixmap.h>
#include <qpainter.h>
#include <qdialog.h>

#include "BoardWidget.h"
#include "../datatypes/ModelPoints.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QLabel;
class QPushButton;

class ModelView : public QDialog
{
    Q_OBJECT
	public:
	    ModelView(QWidget * parent = 0, const char * name = 0, bool modal = FALSE);
	    ~ModelView();
	
	    QLabel* actPoint;
	    QPushButton* previous;
	    QPushButton* deletePoint;
	    QPushButton* next;
	    
	    BoardWidget* getBoard();
	    void updateModelView(int proID,int pIndex);
	    int getActPointIndex();
	    void setActPointIndex(int index);
	
	public slots:
	    void showNextPoint();
	    void showPreviousPoint();
	    void deleteActPoint();
	
	protected:
	    QGridLayout* modelViewLayout;
	    QHBoxLayout* buttonLayout;
	    QSpacerItem* spacer;
	
	protected slots:
	    virtual void languageChange();
	    
	private:
		BoardWidget* board;
		int actPointIndex;
		int projectID;
		int picIndex;
		
};

#endif /*MODELVIEW_H_*/
