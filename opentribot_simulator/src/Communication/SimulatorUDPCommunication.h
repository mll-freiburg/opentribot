
#ifndef _TribotsSim_SimulatorUDPCommunication_h_
#define _TribotsSim_SimulatorUDPCommunication_h_

#include "../Communication/NonspecificTaggedUDPCommunication.h"
#include <vector>

namespace TribotsSim {

  class SimulatorUDPCommunication : public Tribots::NonspecificTaggedUDPCommunication {
  public:
    /** Konstruktor */
    SimulatorUDPCommunication () throw (std::bad_alloc);
    ~SimulatorUDPCommunication () throw ();
    /** 'Bye' senden und Verbindung schliessen */
    void close () throw ();
    /** Verbindung schliessen, ohne 'Bye' zu senden */
    virtual void free_socket () throw ();
    
    // Methoden zum lesen/schreiben/anfordern/Anforderung abfragen bestimmter Informationen
    // Return: true, wenn Information geschrieben werden konnte/empfangen wurde
    virtual bool putPing () throw (std::bad_alloc);
    virtual bool getPing () throw ();

    virtual bool putBye () throw (std::bad_alloc);
    virtual bool getBye () throw ();

    // Roboterposition und -geschwidigkeit; Argumente: x, y, phi, vx, vy, vphi, kicklength
    virtual bool putRobot (double, double, double, double, double, double, bool) throw (std::bad_alloc);
    virtual bool getRobot (double&, double&, double&, double&, double&, double&, bool&) throw ();

    // Ballposition und -geschwidigkeit; Argumente: x, y, z, vx, vy, vz
    virtual bool putBall (double, double, double, double, double, double) throw (std::bad_alloc);
    virtual bool getBall (double&, double&, double&, double&, double&, double&) throw ();
    
    // Fahrtvektor/Odometrie; Argumente: vx, vy, vphi, kicklength
    virtual bool putDriveVector (double, double, double, unsigned int) throw (std::bad_alloc);
    virtual bool getDriveVector (double&, double&, double&, unsigned int&) throw ();

    // Hindernisse: Argumente: x1, y1, x2, y2, x3, y3, ...
    virtual bool putObstacles (const std::vector<double>&) throw (std::bad_alloc);
    virtual bool getObstacles (std::vector<double>&) throw (std::bad_alloc);
  };

}

#endif
