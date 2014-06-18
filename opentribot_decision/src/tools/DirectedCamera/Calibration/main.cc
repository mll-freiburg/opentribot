
#include "../../components/ImageWidget.h"
#include "../../../ImageProcessing/Formation/Image.h"
#include "../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../ImageProcessing/Formation/YUVImage.h"
#include "DirectionalCalibration.h"
#include "../../../Structures/Journal.h"
#include <fstream>

/**
 main.cc
 Provides an interface to the DirectionalCalibration class.
 It reads calibration points (Pixel->Worldcoordinate mappings) from a file.
 Use CalibTool to manually generate mapping files from existing camera images.
 Command line arguments:
  directional_calibration.cfg    -  the output file. This is used by Image2WorldMapping to
                                    retrieve world coordinates from pixels after calibration.
  calibration_points_file        -  manually generated file containing calibration points.
                                    Use this file and not the automated chessboard retrieval.

 @author Christopher Lörken (cloerken@uos.de), Tobias Kringe (tkringe@uos.de), (Sascha Lange) [Martin Lauer]
 @date 16.06.2005, (22.03.2006) [26.5.2006]
*/

using namespace TribotsTools;
using namespace Tribots;
using namespace std;

namespace {
  void printUsage(const char *cmd)
  {
    cerr << "Usage: " << cmd << " 'directional_calibration.cfg' "
        <<"calibration_points_file [-i test_image.ppm]" << endl;
    cerr << "'directional_calibration.cfg': result file with camera parameters\n";
    cerr << "calibration_points_file: file with point-correspondences. Each line\n";
    cerr << "contains four numbers, the world coordinates in the plane and the\n";
    cerr << "corresponding image coordinates\n";
    cerr << "The correspondences belonging to different planes are separated by\n";
    cerr << "A line that contains image width, image height, and two '-1''\n";
    cerr << "If the -i option is given a test image is taken and displayed after\n";
    cerr << "being undistorted and flattened.\n";
  }
}

int main(int argc, char** argv) {
  try{
    if(argc<3){
       printUsage(argv[0]);
       return -1;
    }
    Tribots::Journal::the_journal.set_stream_mode (std::cerr);

    string dest_filename = argv[1];
    string source_filename = argv[2];
    string test_filename = "";

    // Analyse additional options
    int argcount=3;
    while(argc>argcount){
      switch(argv[argcount++][1]){
        case 'i' :
          test_filename=argv[argcount++];
        break;
        default:
          cerr << "Wrong argument: " << argv[argcount] << "endl";
       }
    }

    DirectionalCalibration calibration;
    cout << "Lese Marker aus Datei " << source_filename.c_str() << endl;
    calibration.readMarkers (source_filename.c_str());
    cout << "Kalibriere Kamera" << endl;
    calibration.calibrate ();

    if (dest_filename.length()>0) {
      std::ofstream dest (dest_filename.c_str());
      if (!dest)
        throw std::invalid_argument ("main.cc: Kann Zieldatei nicht oeffnen.");
      calibration.getCameraOptics().writeParametersToStream (dest);
      dest << std::flush;
      cout << "Kameraparameter geschrieben in Datei " << dest_filename << endl;
    } else {
      calibration.getCameraOptics().writeParametersToStream (cout);
    }
UndistortionMapping udmap (calibration.getCameraOptics(), 710, 410);
    if (test_filename.length()>0) {
      QApplication app(argc, argv);

      PPMIO io;
      ifstream imagein (test_filename.c_str());
      Image* testImage = new RGBImage (*io.read (NULL, imagein));
      Image* undistortImage = calibration.undistortImage (testImage);
/*      Image* undistortImage = new RGBImage (testImage->getWidth(), testImage->getHeight());
      UndistortionMapping udmap (calibration.getCameraOptics(), 710, 410);
      for (int u=0; u<undistortImage->getWidth(); u++) {
        for (int v=0; v<undistortImage->getWidth(); v++) {
          RGBTuple rgb;
          testImage->getPixelRGB (u,v,&rgb);
          Vec p = udmap.map (Vec(u,v));
          int u1 = static_cast<int>(p.x+0.5);
          int v1 = static_cast<int>(p.y+0.5);
          if (u1>=0 && v1>=0 && u1<undistortImage->getWidth() && v1<undistortImage->getHeight())
          undistortImage->setPixelRGB (u1, v1, rgb);
        }
      }*/
      Image* flatImage = calibration.flattenImage (testImage);
      ImageWidget undistortWidget (*undistortImage);
      ImageWidget flatWidget (*flatImage);

      undistortWidget.show();
      flatWidget.show();
      app.setMainWidget (&undistortWidget);
      QObject::connect( &app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()) );
      return app.exec();
    }

    return 0;
  }catch(exception& e){
    cerr << "Exception: " << e.what() << endl;
    return -1;
  }
}
