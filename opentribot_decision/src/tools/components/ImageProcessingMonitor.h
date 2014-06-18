#ifndef _ImageProcessingMonitor_h_
#define _ImageProcessingMonitor_h_

#include "ImageWidget.h"
#include "../../ImageProcessing/ObjectAnalysis/ScanLines.h"
#include "../../ImageProcessing/ObjectAnalysis/LineScanning.h"
#include "../../Fundamental/ConfigReader.h"


namespace TribotsTools {

  /** Widget, um die Arbeitsweise der Bildverarbeitung zu visualisieren */
  class ImageProcessingMonitor : public TribotsTools::ImageWidget
  {
  Q_OBJECT

  public:
    ImageProcessingMonitor(const Tribots::ConfigReader& config,
                           const Tribots::Image& image, QWidget* parent = 0,
	   		   WFlags f = WType_TopLevel | WNoAutoErase);

    virtual ~ImageProcessingMonitor();
    
  public slots:
    void setScanLines(const Tribots::ScanLines* scanLines);
    void setScanResults(const Tribots::ScanResultList* scanResults);


  protected:
    void paintEvent(QPaintEvent *ev);

    const Tribots::ScanLines* scanLines;
    const Tribots::ScanResultList* scanResults;

    Tribots::ColorClassInfoList* colClass;
  };			      
}

#endif
