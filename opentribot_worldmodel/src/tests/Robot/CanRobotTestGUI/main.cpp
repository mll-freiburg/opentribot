#include <qapplication.h>
#include "mainform.h"
#include <cstdlib>
int main( int argc, char ** argv )
{
    QApplication a( argc, argv );
    MainForm w;
    
    if (argc < 2) {	
	std::cout << "No config file given, using default : ../../../../config_files/robotNewStd.cfg\n";
	w.set_config_file("../../../../config_files/robotNewStd.cfg");
    }
    else if (QString(argv[1]) == "-d") {
	std::cout << "No config file will be used, default params.\n";
    }
    else if (QString(argv[1]) == "-h") {
	std::cout << "Usage: " << argv[0] << " <> | <cnfig file> | -h | -d\n";
	exit(0);
    }
    else {
	std::cout << "Using config file: " << argv[1] << "\n";
	w.set_config_file(std::string(argv[1]));
    }
    w.show();
    a.connect( &a, SIGNAL( lastWindowClosed() ), &a, SLOT( quit() ) );
    return a.exec();
}
