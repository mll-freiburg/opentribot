
#ifndef _TribotsTools_RGBColorClassifier_h_
#define _TribotsTools_RGBColorClassifier_h_

#include "../../ImageProcessing/Formation/ColorTuples.h"
#include "../../ImageProcessing/PixelAnalysis/ColorClassifier.h"
#include <vector>

namespace TribotsTools {

  class RGBColorClassifier : public Tribots::ColorClassifier {
  private:
      std::vector<unsigned char> rmin, rmax, gmin, gmax, bmin, bmax;

  public:
    RGBColorClassifier ();
    ~RGBColorClassifier ();
    const unsigned char& lookup (const Tribots::RGBTuple&  rgb)  const;

    void clear ();
    void load (std::string filename);
    void save (std::string filename) const;
    void createFromExamples (const std::vector<std::vector<Tribots::RGBTuple> >&);
    Tribots::ColorClassifier* create() const;

    const unsigned char& lookup (const Tribots::YUVTuple&)  const;  ///< nicht implmentiert
    const unsigned char& lookup (const Tribots::UYVYTuple&, int =0) const;  ///< nicht implmentiert
    void set (const Tribots::RGBTuple&, unsigned char) {;}  ///< nicht implmentiert
    void set (const Tribots::YUVTuple&, unsigned char) {;}  ///< nicht implmentiert
    void set (const Tribots::UYVYTuple&, unsigned char, int) {;}  ///< nicht implmentiert
  };

}
#endif
