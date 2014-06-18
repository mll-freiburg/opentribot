#include "WavePlay.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>


using namespace std;
        
Tribots::WavePlay* Tribots::WavePlay::theInstance = 0;

Tribots::WavePlay::WavePlay()
{
  finished = true;
  data_mutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
  theThread=0;
}

Tribots::WavePlay::~WavePlay() {}

Tribots::WavePlay* Tribots::WavePlay::getInstance()
{
  if (theInstance==0) theInstance = new Tribots::WavePlay();
  return theInstance;
}

bool Tribots::WavePlay::isPlaying()
{
  bool res;
  pthread_mutex_lock( &data_mutex );
  res = !finished;
  pthread_mutex_unlock( &data_mutex );
  return res;
}

bool Tribots::WavePlay::startPlaying(int sig)
{
  if (!finished) return false;

  signal = sig;

  pthread_mutex_lock( &data_mutex );
  finished = false;
  pthread_mutex_unlock( &data_mutex );
  if (theThread!=0)
    pthread_join(theThread,NULL);
  int ret = pthread_create( &theThread, NULL, wavePlayThread, (void*) this);
  //std::cerr << "Starte thread mit: " << ret << "\n";
  return (ret == 0);
}

void* Tribots::WavePlay::wavePlayThread(void *WavePlay_instance) {
  WavePlay* inst = (WavePlay*) WavePlay_instance; 

  string command="aplay";
  string filename ="";
  
  switch (inst->signal) {
    case 1:
      filename = "../src/Audio/breep.wav";
      break;
    case 2:
      filename = "../src/Audio/breep2.wav";
      break;
    case 3:
      filename = "../src/Audio/R2D2c.wav";
      break;
    default:
      break;
  }

  if (filename.length() > 0) {
    std::ostringstream o;
    o << command << " " << filename;
    int i = system(o.str().c_str());  
    printf ("The value returned was: %d.\n",i);
  }
  
  pthread_mutex_lock( &inst->data_mutex );
  inst->finished = true;
  pthread_mutex_unlock( &inst->data_mutex );

  pthread_exit(NULL);
}
