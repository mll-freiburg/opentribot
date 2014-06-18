#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"


namespace Tribots {

  class BStayInsideArea : public Behavior {
  public:
    /** Konstruktor. 
     *
     * \param area convexer Bereich, in dem der Roboter bleiben soll.
     * \param target Mittelpunkt des Bereichs, bzw. der Punkt, den der Roboter anfahren 
     *               soll, solange er sich auﬂerhalb des Bereichs befindet.
     */
    BStayInsideArea(const Area& area, Vec target);
    virtual ~BStayInsideArea() throw ();

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);    

  protected:
    Area* area;
    Vec target;
  };

}
