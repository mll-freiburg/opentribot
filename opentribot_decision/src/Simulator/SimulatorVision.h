
#ifndef _Tribots_SimulatorVision_h_
#define _Tribots_SimulatorVision_h_

#include "../ImageProcessing/VisionType.h"
#include "../Fundamental/geometry.h"
#include "SimClient.h"
#include "../Fundamental/ConfigReader.h"
#include <vector>
#include <fstream>

namespace Tribots {

  /** Klasse SimulatorImageProcessing erzeugt eine virtuelle Bildverarbeitung,
      die ihre Eingaben aus dem Simulator liest und Tore sowie weiﬂe Linien liefert */
  class SimulatorVision : public VisionType {
  private:
   SimClient* the_sim_client;         ///< pointer auf den SimClient
   double noise_level;                ///< Standardabweichung fuer Rauschen
   double line_mis_probability;       ///< Wahrscheinlichkeit, dass ein Sensorwert fehlt
   double goal_mis_probability;       ///< Wahrscheinlichkeit, dass ein Sensorwert fehlt
   double obstacle_mis_probability;   ///< Wahrscheinlichkeit, dass ein Sensorwert fehlt
   double ball_mis_probability;       ///< Wahrscheinlichkeit, dass Ball nicht gefunden wird
   double line_vision_radius;         ///< Radius, bis zu dem Linien erkannt werden
   double ball_vision_radius;         ///< analog fuer Ball
   double obstacle_vision_radius;     ///< analog fuer Hindernisse
   std::vector<LineSegment> lines;     ///< Spielfeldlinien
   std::vector<Arc> arcs;              ///< Spielfeldlinien
   std::vector<Line> scanlines;        ///< die Scanlinien (beidseitig!)

   bool simulate_gyro;   ///< Gyroskop simulieren?
   bool simulate_odo;   ///< Odometrie simulieren?
   double odo_x_noise;
   double odo_y_noise;
   double odo_phi_noise;
   double gyro_phi_noise;

   std::ofstream* logtruestream;
  public:
   /** Konstruktor */
   SimulatorVision (const ConfigReader&) throw (std::bad_alloc, Tribots::InvalidConfigurationException);
   /** Destruktor */
   ~SimulatorVision () throw ();
   
   int get_num_sources() const throw() { return 1; }  
   /** virtuelle Sensorinformationen liefern */
   void process_images () throw ();
  };

}

#endif
