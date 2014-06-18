#include <qapplication.h>
#include "Monitor.h"

int main( int argc, char ** argv )
{
    QApplication a( argc, argv );
    MonitorForm w;
    w.show();
    a.connect( &a, SIGNAL( lastWindowClosed() ), &a, SLOT( quit() ) );
    return a.exec();
}
