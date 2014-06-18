#include "ModelView.h"
#include "BoardWidget.h"
#include "MainWindow.h"
#include "ImageClickWidget.h"
#include "../projectmanagement/ProjectManager.h"
#include <qpushbutton.h>
#include <qevent.h>
#include <qframe.h>
#include <qlayout.h>
#include <qpixmap.h>
#include <qpainter.h>
#include <qvariant.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qsize.h>


ModelView::ModelView(QWidget * parent, const char * name, bool modal)
    : QDialog(parent, "ModelView", modal)
{
	actPointIndex = 0;
	
	setFixedSize( QSize( 400, 300 ) );    

    modelViewLayout = new QGridLayout( this, 1, 1, 11, 6, "modelViewLayout"); 
    modelViewLayout->setResizeMode(QLayout::FreeResize);
    buttonLayout = new QHBoxLayout( 0, 2, 2, "buttonLayout"); 
    buttonLayout->addStretch();

    actPoint = new QLabel( this, "actPoint" );
    buttonLayout->addWidget( actPoint );
    
    spacer = new QSpacerItem( 21, 20, QSizePolicy::Expanding, QSizePolicy::Minimum );
    buttonLayout->addItem( spacer );

    previous = new QPushButton( this, "previous" );
    previous->resize( QSize(100,30) );
    buttonLayout->addWidget( previous );

    deletePoint = new QPushButton( this, "delete" );
    deletePoint->setEnabled( FALSE );
    deletePoint->resize( QSize(100,30) );
    buttonLayout->addWidget( deletePoint );

    next = new QPushButton( this, "next" );
    next->resize( QSize(100,30) );
    buttonLayout->addWidget( next );

    modelViewLayout->addLayout( buttonLayout, 1, 0 );

    board = new BoardWidget( this, "board" );
    modelViewLayout->addWidget( board, 0, 0 );
    
    
    

    languageChange();

    resize( QSize(400, 300).expandedTo(minimumSizeHint()) );
    clearWState( WState_Polished );

    QObject::connect(previous, SIGNAL(clicked()), this, SLOT(showPreviousPoint()));
    QObject::connect(deletePoint, SIGNAL(clicked()), this, SLOT(deleteActPoint()));
    QObject::connect(next, SIGNAL(clicked()), this, SLOT(showNextPoint()));

}

ModelView::~ModelView()
{
}

void ModelView::showNextPoint() 
{
	actPointIndex++;
	if(actPointIndex == ((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints().size()) { 
		actPointIndex = 0;
	}
    board->drawModelPoints(projectID, picIndex, actPointIndex);
    ModelPoints temp = (((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints()).at(actPointIndex);
    if(temp.getClicked()) {
    	deletePoint->setEnabled( true );
    	actPoint->setText(QString("Point ( %1, %2, 1)").arg(temp.getXModel()).arg(temp.getYModel()));
    } else {
    	deletePoint->setEnabled( false );
    	actPoint->setText(QString("Point ( %1, %2, 0)").arg(temp.getXModel()).arg(temp.getYModel()));
    }
}

void ModelView::showPreviousPoint()
{
	actPointIndex--;
	if(actPointIndex == -1) { actPointIndex = ((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints().size()-1;}
	board->drawModelPoints(projectID, picIndex, actPointIndex);
	ModelPoints temp = (((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints()).at(actPointIndex);
	if(temp.getClicked()) {
		deletePoint->setEnabled( true );
    	actPoint->setText(QString("Point ( %1, %2, 1)").arg(temp.getXModel()).arg(temp.getYModel()));
    } else {
    	deletePoint->setEnabled( false );
    	actPoint->setText(QString("Point ( %1, %2, 0)").arg(temp.getXModel()).arg(temp.getYModel()));
    }
}

void ModelView::deleteActPoint()
{
	// Index des aktuellen Punktes besorgen
   	int index = getActPointIndex();
   	
   	// ProjektID besorgen
   	int proID = ProjectManager::getActProjectID();
   		
	// ModelPoints bearbeiten
	((((ProjectManager::getProject(proID)).getCalibImage(picIndex)).getModelPoints()).at(index)).setClicked(false);
	((((ProjectManager::getProject(proID)).getCalibImage(picIndex)).getModelPoints()).at(index)).setXWorld(-1.0);
		((((ProjectManager::getProject(proID)).getCalibImage(picIndex)).getModelPoints()).at(index)).setYWorld(-1.0);

   	// ModelView aktualisieren
   	updateModelView(proID, picIndex);
   	
   	// Image bearbeiten
   	MainWindow* mainwindowtemp = dynamic_cast<MainWindow*>(parentWidget(false));
   	ImageClickWidget* imageclickwidgettemp = dynamic_cast<ImageClickWidget*>(mainwindowtemp->mainwidget->tabwidget->currentPage());
   	imageclickwidgettemp->markSelections();
   	imageclickwidgettemp->setClickCount(imageclickwidgettemp->getClickCount()-1);
   	
}

int ModelView::getActPointIndex() { return actPointIndex; }

void ModelView::setActPointIndex(int index) 
{
	actPointIndex = index;	
}

void ModelView::updateModelView(int proID, int pIndex)
{
	projectID = proID;
	picIndex = pIndex;
	board->drawModelPoints(projectID, picIndex, actPointIndex);
	ModelPoints temp = (((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints()).at(actPointIndex);
    if(temp.getClicked()) {
    	deletePoint->setEnabled( true );
    	actPoint->setText(QString("Point ( %1, %2, 1)").arg(temp.getXModel()).arg(temp.getYModel()));
    } else {
    	deletePoint->setEnabled( false );
    	actPoint->setText(QString("Point ( %1, %2, 0)").arg(temp.getXModel()).arg(temp.getYModel()));
    }
    // clickCount neu setzen
    // bei oeffnen eines bestehenden Projects ist clickCount des ImageClickWidgets auf null, obwohl vielleicht schon Punkte gesetzt worden
    std::vector<ModelPoints> v;
    int count = 0;
    v = ((ProjectManager::getProject(projectID)).getCalibImages()).at(picIndex).getModelPoints();
    for(unsigned int i = 0; i < v.size(); i++ ) {
    	if((v.at(i)).getClicked()) {
    		count++;	
    	}
    }
    // Image bearbeiten
   	MainWindow* mainwindowtemp = dynamic_cast<MainWindow*>(parentWidget(false));
   	ImageClickWidget* imageclickwidgettemp = dynamic_cast<ImageClickWidget*>(mainwindowtemp->mainwidget->tabwidget->currentPage());
   	imageclickwidgettemp->setClickCount(count);
}

BoardWidget* ModelView::getBoard() { return board;}

/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void ModelView::languageChange()
{
    setCaption( tr( "ModelView" ) );
    actPoint->setText( tr( "actPoint" ) );
    previous->setText( tr( "<<" ) );
    deletePoint->setText( tr( "delete" ) );
    next->setText( tr( ">>" ) );
}

