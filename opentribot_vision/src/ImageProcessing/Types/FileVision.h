
#ifndef _Tribots_FileVision_h_
#define _Tribots_FileVision_h_

#include "../VisionType.h"
#include "../../Structures/VisibleObjectReadWriter.h"
#include "../../Fundamental/ConfigReader.h"
#include <fstream>

namespace Tribots {

  /** Klasse FileImageProcessing liest Information ueber erkannte Objekte 
      aus einer Datei */
  class FileVision : public VisionType {
  public:
    /** Konstruktor */
    FileVision (const ConfigReader&) throw (std::bad_alloc, Tribots::InvalidConfigurationException);
    /** Destruktor */
    ~FileVision () throw ();
    
    int get_num_sources() const throw() { return 1; }
    /** weitere Objekte aus Datei auslesen */
    void process_images () throw ();

  private:
    std::ifstream* stream;
    VisibleObjectReader* reader;
    bool cut_outside_field;
  };

}

#endif
