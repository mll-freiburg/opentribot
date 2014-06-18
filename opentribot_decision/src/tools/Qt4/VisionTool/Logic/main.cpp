
#include <QtGui/QApplication>
#include <iostream>
#include "../Widgets/VisionToolMainWindow.h"
#include "../Logic/VisionToolImageSource.h"

using namespace TribotsTools;
using namespace std;

int main (int argc, char *argv[]) {

#ifdef Q_OS_MAC
cout << "compiled for OSX" << endl;
#endif

  try{
    QApplication app (argc,argv);

    Tribots::ConfigReader config (1);
    std::string section = "XXX";
    if (argc>=3) {
      config.append_from_file (argv[1]);
      section = argv[2];
    }
    config.set ("VisionTool::Section", section);
    VisionToolImageSource is (config);
    try{
      std::string kind="";
      config.get ((section+"::image_source_type").c_str(),kind);
      if (kind=="FileSource")
        is.startFileSource ();
      else if (kind=="CameraSource")
        is.startCameraSource ();
    }catch(Tribots::TribotsException& e) {
      std::cerr << "Bildquelle noch nicht bereit.\n";
    }
    VisionToolMainWindow mw (is, config);
    mw.show();
    mw.loop();

    config.write_section (std::cout, section.c_str());

    return 0;
  }catch (Tribots::TribotsException& e) {
    std::cerr << e.what() << '\n';
    return -1;
  }catch (std::exception& e) {
    std::cerr << e.what() << '\n';
    return -1;
  }
}
