
#include "POSIXThread.h"
#include <errno.h>

namespace {
  void* startPThread(void* arg) {
    static_cast<Tribots::POSIXThread*>(arg)->mainCall();
    return 0;
  }
}

Tribots::POSIXThread::POSIXThread () {
  threadRunning=false;
}

Tribots::POSIXThread::~POSIXThread() {
  exit();
}

void Tribots::POSIXThread::mainCall () {
  main ();
}

void Tribots::POSIXThread::start() {
  bool tr;
  im.lock();
  tr=threadRunning;
  im.unlock();
  if (!threadRunning) {
    cancelSet=false;
    threadRunning=true;
    pthread_create(&id, 0, startPThread, this);
  }
}

void Tribots::POSIXThread::exit() {
  bool tr=true;
  im.lock();
  tr=threadRunning;
  threadRunning=false;
  im.unlock();
  if (tr)
    pthread_exit(NULL);
}

void Tribots::POSIXThread::waitForExit () {
  bool rn;
  im.lock();
  rn=threadRunning;
  im.unlock();
  if (rn)
    pthread_join(id, NULL);
}

void Tribots::POSIXThread::cancel () {
  im.lock();
  cancelSet=true;
  im.unlock();
}

void Tribots::POSIXThread::checkCancel () {
  bool cs, tr;
  im.lock();
  cs=cancelSet;
  tr=threadRunning;
  threadRunning=false;
  im.unlock();
  if (cs && tr) {
    threadCanceled();
    pthread_exit(NULL);
  }
}

bool Tribots::POSIXThread::running () {
  bool rn;
  im.lock();
  rn=threadRunning;
  im.unlock();
  return rn;
}


Tribots::POSIXMutex::POSIXMutex () throw () {
  pthread_mutex_init (&m, NULL);
}

Tribots::POSIXMutex::~POSIXMutex () throw () {
  pthread_mutex_destroy (&m);
}

void Tribots::POSIXMutex::lock () throw () {
  pthread_mutex_lock (&m);
}

bool Tribots::POSIXMutex::trylock () throw () {
  return (pthread_mutex_trylock (&m)!=EBUSY);
}

void Tribots::POSIXMutex::unlock () throw () {
  pthread_mutex_unlock (&m);
}


Tribots::POSIXConditional::POSIXConditional () throw () {
  pthread_mutex_init (&m, NULL);
  pthread_cond_init (&c, NULL);
}

Tribots::POSIXConditional::~POSIXConditional () throw () {
  pthread_cond_destroy (&c);
  pthread_mutex_destroy (&m);
}

void Tribots::POSIXConditional::wait () throw () {
  pthread_mutex_lock (&m);
  pthread_cond_wait (&c, &m);
  pthread_mutex_unlock (&m);
}

void Tribots::POSIXConditional::signal () throw () {
  pthread_mutex_lock (&m);
  pthread_cond_signal (&c);
  pthread_mutex_unlock (&m);
}

void Tribots::POSIXConditional::broadcast () throw () {
  pthread_mutex_lock (&m);
  pthread_cond_broadcast (&c);
  pthread_mutex_unlock (&m);
}
