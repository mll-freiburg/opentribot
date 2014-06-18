#include <iostream>

#include "../../tools/components/ImageProcessingMonitor.h"
#include "../../tools/components/ImageWidget.h"
#include "../../ImageProcessing/Formation/ImageProducer.h"
#include "../../ImageProcessing/Formation/MultiImageProducer.h"
#include "../../ImageProcessing/Formation/Image.h"
#include "../../ImageProcessing/Formation/RGBImage.h"
#include "../../ImageProcessing/Formation/YUVImage.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../ImageProcessing/ObjectAnalysis/ScanLines.h"
#include "../../ImageProcessing/PixelAnalysis/Image2WorldMapping.h"
#include "../../Fundamental/Vec.h"
#include "../../Structures/Journal.h"
#include "SavingObject.h"
#include "../../ImageProcessing/Formation/PPMIO.h"

#include <sstream>
#include <fstream>


using namespace Tribots;
using namespace std;


int main(int argc, char* argv[])
{
  string cfg = "../config_files/robotcontrol.cfg";
  if (argc>=2 && (string(argv[1])=="--help")) {
    cerr << "Aufruf: " << argv[0] << " [Konfigurationsdatei=../config_files/robotcontrol.cfg]" << endl;
    return -1;
  }
  if (argc>=2)
    cfg = argv[1];

  try{
    ConfigReader config(0);
    config.append_from_file(cfg.c_str());
    Tribots::Journal::the_journal.set_stream_mode (std::cerr);

    //Support for multiple image sources:
    MultiImageProducer producer(config);
    int num_sources = producer.numSources();

    //---Initializations
    Image* image[num_sources];
    TribotsTools::ImageWidget* widgets[num_sources];
    //Initialize QApplication
    QApplication app(argc, argv);	//only one QApplication is allowed.
    TribotsTools::SavingObject saver;

    for (int i = 0; i < num_sources; i++){
      try{
        image[i] = producer.nextImage(i); //load first images from different sources
      }catch(ImageFailed){
        image[i] = new RGBImage (10, 10);
      }
      widgets[i] = new TribotsTools::ImageWidget(*image[i]);
      widgets[i]->show();
      QObject::connect ( widgets[i], SIGNAL(keyPressed(QKeyEvent*)), &saver, SLOT(acceptKey(QKeyEvent*)));
    }
    app.setMainWidget(widgets[0]);

    QObject::connect( &app, SIGNAL(lastWindowClosed()),
                      &app, SLOT(quit()) );

    //Main loop that shows the images.
    //Active until window is closed.

    //multiple images
    Image* newImage[num_sources];

    cout << "Type [s] to save images from all cameras" << endl;
    unsigned int saveCounter=0;
    unsigned int doSave=0;
    while (app.mainWidget()->isVisible()) {
      if (saver.check())
        doSave=num_sources+1;

      for (int i = 0; i < num_sources; i++){
        try{
          newImage[i] = producer.nextImage(i); //load image from currentsources
          //war vorher am Ende:
          widgets[i]->setImage(*newImage[i]);

          if (doSave) {
            // Bilder abspeichern:
            PPMIO io;
            stringstream inout;
            inout << "image_n" << saveCounter << "_c" << i << ".ppm";
            string filename = inout.str();
            ofstream os (filename.c_str());
            io.write (newImage[i]->getImageBuffer(), os);
            os << flush;
            doSave--;
            cerr << "saving image in file " << filename << endl;
          }
          delete newImage[i];
        }catch(ImageFailed&){;} // falls kein Bild bekommen, nichts tun

        app.processEvents();

      }//end for all images
      if (doSave==1) {
        saveCounter++;
        doSave=0;
      } else if (doSave>1) {
        doSave=num_sources+1;
      }
    }  //end while loop

  }catch(Tribots::InvalidConfigurationException& e){
    cerr << e.what() << "\n\r" << flush;
  }catch(Tribots::TribotsException& e){
    cerr << e.what() << "\n\r" << flush;
  }catch(std::exception& e){
    cerr << e.what() << "\n\r" << flush;
  }
  return 0;
}
