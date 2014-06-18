#ifndef _MorphologicOperators_h_
#define _MorphologicOperators_h_

#include "../../ImageProcessing/Formation/Image.h"

namespace Tribots {

  /**
   * Interface f�r Operatoren, die ein Bild bearbeiten und ein neues, 
   * ver�ndertes Bild zur�ckliefern.
   */
  class ImageOperator {
  public:
    /**
     * Virtueller Destrkutor.
     */
    virtual ~ImageOperator() {};
    
    /**
     * Liefert ein neues, ver�ndertes Bild. Der Speicher geht in den Besitz
     * des Aufrufers �ber.
     *
     * \param image Ursprungsbild
     * \return Ein neu erzeugtes Image, das das Ergebnis enth�lt.
     */
    virtual Image* operator() (const Image&) const =0;
  };

  /**
   * Dilatationsoperator. Vergr��ert weisse Fl�chen um die angrenzenden Pixel.
   */
  class Dilatation : public ImageOperator {
  public:
    Dilatation(int size = 3);
    
    virtual ~Dilatation();
    virtual Image* operator() (const Image&) const;

  protected:
    int size;
  };

  /**
   * Erosionsoperator. Verkleinert weisse Fl�chen um Randpixel.
   */
  class Erosion : public ImageOperator {
  public:
    Erosion(int size = 3);
    
    virtual ~Erosion();
    virtual Image* operator() (const Image&) const;

  protected:
    int size;
  };


}

#endif
