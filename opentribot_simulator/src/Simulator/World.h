
#ifndef _TribotsSim_World_h_
#define _TribotsSim_World_h_

#include <ode/ode.h>
#include <vector>

namespace TribotsSim {

  /** Klasse, die allgemein ein Objekt beschreibt */
  class GeneralObject {
  public:
    virtual ~GeneralObject () {;}
    virtual void create (dWorldID, dSpaceID) =0;
    virtual void draw () =0;
    
    virtual const dReal* getPosition () const;
    virtual const dReal* getLinearVelocity () const;
    virtual const dReal* getRotation () const;
    virtual const dReal* getAngularVelocity () const;
    virtual void setPosition (dReal, dReal, dReal);
    virtual void setLinearVelocity (dReal, dReal, dReal);
    virtual void setRotation (dReal, dReal, dReal);
    virtual void setAngularVelocity (dReal, dReal, dReal);
    virtual void addForce (dReal, dReal, dReal);
    virtual void addRelativeForce (dReal, dReal, dReal);
    virtual void addTorque (dReal, dReal, dReal);
    virtual void addRelativeTorque (dReal, dReal, dReal);
    
  protected:
    dBodyID body;
  };
  

  /** Klasse, die einen simulierten Tribot realisiert */
  class Tribot : public GeneralObject {
  public:
    Tribot ();
    ~Tribot ();

    void create (dWorldID, dSpaceID);   ///< Roboter erzeugen
    void draw  ();  ///< Roboter zeichnen
    void setActive (bool);  ///< als aktiven (handgesteuerten) Roboter kennzeichnen
    bool isActive () const;
    
    void makeSnapshot (std::vector<dReal>&);
    unsigned int recoverSnapshot (const std::vector<dReal>&, unsigned int);

    void setSteeringX (dReal);
    void setSteeringY (dReal);
    void setSteeringPhi (dReal);
    void setSteeringKick (bool);
    void setNoSteering ();
    const dReal* getSteering () const;
    
  protected:
    dGeomID geom [4];  ///< Einzelteile des Roboters
    unsigned int num_geom;  ///< Anzahl Einzelteile
    bool is_active;
    dReal steering [4];  ///< x,y-Geschwindigkeit, Rotationsgeschwindigkeit und kicken
  };

  /** Klasse, die den simulierten Ball realisiert */
  class Ball : public GeneralObject {
  public:
    Ball ();
    ~Ball ();

    void create (dWorldID, dSpaceID);   ///< Ball erzeugen
    void draw  ();  ///< Ball zeichnen
    void setActive (bool);  ///< als aktiv (handgesteuert) kennzeichnen
    bool isActive () const;
    
    void makeSnapshot (std::vector<dReal>&);
    unsigned int recoverSnapshot (const std::vector<dReal>&, unsigned int);

  protected:
    dGeomID geom;
    bool is_active;
  };

  class Goal {
  public:
    Goal ();
    ~Goal ();

    void create (dWorldID, dSpaceID, int);   ///< Tor erzeugen, arg3=Seite (-1=gelb, +1=blau)
    void draw  ();  ///< Tor zeichnen
  protected:
    dGeomID geom [3];  ///< Einzelteile des Tors
    unsigned int num_geom;
    int side;  ///< +1=blau, -1=gelb
  };

  class Field : public GeneralObject {
  public:
    Field ();
    ~Field ();
    
    void create (dWorldID, dSpaceID);
    void draw ();
  };

  struct World {
    World ();
    ~World ();
    
    void draw ();
    
    dWorldID wid;   ///< ODE-Welt
    dSpaceID sid;   ///< ODE-Space
    dJointGroupID cid;   ///< ODE-Contactgroup
    dGeomID gid;  ///< Boden
    
    Field field;
    std::vector<Goal> goals;
    std::vector<Tribot> robots;
    Ball ball;
    
    void makeSnapshot (std::vector<dReal>&);  ///< einen Schnappschuss der Feldkonfiguration anlegen und in arg1 zurueckgeben
    unsigned int recoverSnapshot (const std::vector<dReal>&, unsigned int =0);   ///< einen zuvor gefertigten Schnappschuss der Feldkonfiguration setzen, lese ab Position (arg2); wirft invalid_argument bei Fehler; liefert Index des ersten nicht mehr gelesenen Feldes zurueck
  };

  void saveSnapshot (const std::vector<dReal>&, const char*);
  void loadSnapshot (std::vector<dReal>&, const char*);

}

#endif
