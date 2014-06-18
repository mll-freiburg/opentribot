
#ifndef _Tribots_GrayLevelImage_h_
#define _Tribots_GrayLevelImage_h_

#include "../../Fundamental/Table.h"
#include "../Formation/Image.h"

namespace Tribots {

  /** Grauwertbild mit reellwertigen Eintraegen, insbesondere fuer Faltungen
      usw. geeignet. */
  class GrayLevelImage : public Table<float> {
  public:
    GrayLevelImage (unsigned int w, unsigned int h);
    GrayLevelImage (const GrayLevelImage&);
    GrayLevelImage (const Image&);
    void toImage (Image& dest, float offset=0, float scaling=1) const;
    inline unsigned int width () const { return Table<float>::rows(); }
    inline unsigned int height () const { return Table<float>::columns(); }
  };

}

#endif
