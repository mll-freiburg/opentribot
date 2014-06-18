#include "MainWidget.h"
#include "MainWindow.h"
#include "ImageClickWidget.h"
#include "../../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/Formation/Image.h"
#include "../../../../ImageProcessing/Formation/YUVImage.h"
#include "../../../../ImageProcessing/Formation/ImageIO.h"

#define U_DIM 128
#define V_DIM 128
#define SCALE 5

#include <iostream>
#include <fstream>

#include <qvariant.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qtabwidget.h>
#include <qlayout.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qimage.h>
#include <qpixmap.h>
#include <qscrollview.h>

using namespace std;

/*
 *  Constructs a MainWidget as a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'.
 */
MainWidget::MainWidget( QWidget* parent, const char* name, WFlags fl )
    : QWidget( parent, name, fl )
{
    if ( !name )
	setName( "MainWidget" );
    
    mainlayout = new QGridLayout( this, 1, 1, 5, -1, "mainlayout"); 
    tabwidget = new QTabWidget( this, "tabwidget" );
    
    tabwidget->setMaximumSize(QSize(640,480));
    tabwidget->setMinimumSize(QSize(640,480));
     
    mainlayout->addWidget(tabwidget,0,0,Qt::AlignCenter);
   
    languageChange();
    
    clearWState( WState_Polished );
    
    connect( tabwidget, SIGNAL(currentChanged(QWidget*)), this, SLOT( tabchanged() ) ) ;   
}

/*
 *  Destroys the object and frees any allocated resources
 */
MainWidget::~MainWidget()
{
    // no need to delete child widgets, Qt does it all for us
}


/*
 *  Sets the strings of the subwidgets using the current
 *  language.
 */
void MainWidget::languageChange()
{
    setCaption( tr( "MainWidget" ) );
}


void MainWidget::tabchanged() 
{
	MainWindow* mainwindowtemp = dynamic_cast<MainWindow*>(parentWidget(true));
	mainwindowtemp->modelview->setActPointIndex(0);	
	mainwindowtemp->modelview->updateModelView(ProjectManager::getActProjectID(), tabwidget->currentPageIndex());
}


void MainWidget::removeAllPages()
{	
	int count = tabwidget->count();
	int i = 0;
	while(i < count) {
		tabwidget->removePage(tabwidget->page(0));
		i++;
	}	
}

