#ifndef _WAVEPLAY_H_
#define _WAVEPLAY_H_

#include <pthread.h>

namespace Tribots {

  class WavePlay {
  public:
    static WavePlay* getInstance();
    ~WavePlay();

    bool startPlaying(int signal);
    bool isPlaying();
    int			 signal;

  protected:
    WavePlay();
    static void* wavePlayThread(void *WavePlay_instance);
    
    static WavePlay*  theInstance;
    bool                 finished;
    pthread_mutex_t      data_mutex;
    pthread_t            theThread;
  };


}

#endif
