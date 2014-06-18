#include "topwindow.h"

int main( int argc, char **argv )
{
   QApplication a( argc, argv );

   topWindow w;

   a.setMainWidget( &w );
   w.showMaximized();
   return a.exec();
}

