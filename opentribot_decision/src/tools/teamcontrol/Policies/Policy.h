
#ifndef TribotsTools_Policy_h
#define TribotsTools_Policy_h


namespace TribotsTools {

  /** Rollenwechselstrategie, abstrakt */  
  class Policy {
  public:
    virtual ~Policy () throw () {;}

    /** Name der Strategie abfragen */
    virtual const char* get_name () const throw () =0;

    /** eine Funktion, die die Situation beobachtet, ohne einzugreifen */
    virtual void observe () throw () {;}

    /** Strategie anwenden, d.h. Rollenwechsel durchfuehren */
    virtual void update () throw () =0;
  };

}

#endif
