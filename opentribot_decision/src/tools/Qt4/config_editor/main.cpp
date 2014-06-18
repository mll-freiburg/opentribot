
#include <QtGui/QApplication>
#include "ConfigEditorMainWindow.h"

int main( int argc, char ** argv ) {
    QApplication a( argc, argv );
    TribotsTools::ConfigEditorMainWindow* mw = new TribotsTools::ConfigEditorMainWindow();
    mw->show();
    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    return a.exec();
}
