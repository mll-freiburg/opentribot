
#ifndef _Tribots_POSIXThread_h_
#define _Tribots_POSIXThread_h_

#include <pthread.h>

namespace Tribots {


  /** Ein wartender Mutex (Sperre) */
  class POSIXMutex {
  public:
    POSIXMutex () throw ();
    ~POSIXMutex() throw ();
    /** Mutex sperren; wenn Mutex bereits gesperrt, warten bis Mutex entsperrt wurde */
    void lock () throw ();
    /** Mutex pruefen und ggf. sperren: 
      wenn Mutex nicht gesperrt, sperre ihn und liefere true,
      wenn Mutex gesperrt, liefere false ohne den Mutex zu veraendern */
    bool trylock () throw ();
    /** Mutex entsperren */
    void unlock () throw ();
  private:
    pthread_mutex_t m;
  };


    /** eine Thread-Signalisierung */
    class POSIXConditional {
    public:
      POSIXConditional () throw ();
      ~POSIXConditional () throw ();

      /** warte, bis das Signal gegeben wurde */
      void wait () throw ();
      /** sende das Signal (nur ein Empfaenger) */
      void signal () throw ();
      /** sende das Signal (mehrere Empfaenger ) */
      void broadcast () throw ();
    private:
      pthread_mutex_t m;
      pthread_cond_t c;
  };


  /** Thread-Basisklasse basierend auf pthread mit Ergaenzungen (cancel) */
  class POSIXThread {
  public:
    POSIXThread ();
    virtual ~POSIXThread ();

    /** den Thread von aussen starten */
    virtual void start ();
    /** den Thread von aussen auffordern, sich zu beenden */
    virtual void cancel ();
    /** von aussen warten, bis der Thread sich beendet hat */
    virtual void waitForExit ();
    /** von aussen pruefen, ob der Thread laeuft oder nicht */
    virtual bool running ();

    /** Methode aus technischen Gründen notwendig; nicht ueberschreiben oder aufrufen! */
    virtual void mainCall ();
  protected:
    /** die Kernroutine des Threads, diese Problem-spezifisch ueberschreiben */
    virtual void main () =0;
    /** diese Funktion wird am Cancel-Punkt aufgerufen, falls der Thread sich beenden soll */
    virtual void threadCanceled () {};
    
    /** den Thread sich beenden lassen (wird bei cancel nicht aufgerufen) */
    virtual void exit ();
    /** einen Cancel-Punkt definieren, an dem sich der Thread auf Anfrage von aussen aufgeben darf;
          ohne diese Aufrufe sind cancel()-Aufrufe wirkungslos */
    virtual void checkCancel ();

  private:
    pthread_t id;  ///< Thread ID
    bool cancelSet;
    bool threadRunning;
    POSIXMutex im;
  };

}

#endif
