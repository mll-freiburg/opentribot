
#include <qapplication.h>
#include "mainWidget.h"

int main( int argc, char ** argv ) {
    QApplication a( argc, argv );
    mainWidget * mw = new mainWidget();
    mw->setCaption( "QTConfig" );
    mw->showMaximized();
    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    return a.exec();
}
