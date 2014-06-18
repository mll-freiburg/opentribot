#include <iostream>
#include <qapplication.h>
#include "gui/ModelView.h"
#include "gui/MainWindow.h"

using namespace std;

int main( int argc, char ** argv )
{
  QApplication app( argc, argv );
  
  MainWindow *mainwindow = new MainWindow;
  mainwindow->show();
 
  app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );
  return app.exec();

}