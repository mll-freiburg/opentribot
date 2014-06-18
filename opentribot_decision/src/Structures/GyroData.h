
#ifndef _Tribots_GyroData_h_
#define _Tribots_GyroData_h_

namespace Tribots {

  /** Struktur, um die aufbereiteten Messwerte eines Gyroskops zu repraesentieren.
      Angaben in Roboterkoordinaten */
  struct GyroData {
    /** Default-Konstruktor */
    GyroData () throw () : vrot(0) {;}
    /** Copy-Konstruktor */
    GyroData (const GyroData& gd) throw () : vrot(gd.vrot) {;}
    /** Zuweisungsoperator */
    const GyroData& operator= (const GyroData& gd) throw () { vrot=gd.vrot; return *this; }

    double vrot;  ///< Rotationsgeschwindigkeit in rad/s
  };

}

#endif
