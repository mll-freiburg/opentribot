#ifndef _MUTEXED_RING_BUFFER_H_
#define _MUTEXED_RING_BUFFER_H_

#include <sys/time.h>

namespace RobotCtr2 {

// template class for a mutexed ring buffer
template <class T>
class MutexedRingBuffer {
 public:
  MutexedRingBuffer(unsigned int _buffer_size = 20) throw (std::bad_alloc);
  ~MutexedRingBuffer() throw();

  void add(const T& obj);
  void clear();

  bool getLast(T& obj);
  bool getNext(T& res, unsigned long usec_patience);
  void getAll(std::vector< T >& res);
  bool getNew(std::vector< T >& res);

 protected:
  unsigned int         buffer_size;
  unsigned int         windex;
  unsigned int         rindex;
  T                   *buffer;
  
  pthread_mutex_t      buffermutex;

  // Mutexes installed for user waiting on new data (condition mutex) used in -> getNext()
  pthread_mutex_t condition_mutex;
  pthread_cond_t  condition_cond;

};


// -------------- Mutexed Ring Buffer ----------------------------------------------
template <class T>
MutexedRingBuffer<T>::MutexedRingBuffer(unsigned int _buffer_size ) throw (std::bad_alloc)
  : buffer_size( _buffer_size ), windex(0), rindex(0)
{
  buffer = 0;
  buffer = new T[buffer_size];

  buffermutex     = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;

  condition_mutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
  condition_cond  = (pthread_cond_t)  PTHREAD_COND_INITIALIZER;
}

template <class T>
MutexedRingBuffer<T>::~MutexedRingBuffer() throw()
{
  pthread_mutex_lock( &buffermutex );
  if (buffer!=0) delete[] buffer;
  pthread_mutex_unlock( &buffermutex );
}

template <class T>
void MutexedRingBuffer<T>::add(const T& obj)
{
  pthread_mutex_lock( &buffermutex );
  windex = (windex + 1) % buffer_size;
  buffer[windex] = obj;
  pthread_mutex_unlock( &buffermutex );
  // signal waiting getNext() calls that a new data has arrived
  pthread_mutex_lock( &condition_mutex );
  pthread_cond_signal( &condition_cond );
  pthread_mutex_unlock( &condition_mutex );
}

template <class T>
void MutexedRingBuffer<T>::clear()
{
  pthread_mutex_lock( &buffermutex );
  rindex = windex;
  pthread_mutex_unlock( &buffermutex );
}

template <class T>
bool MutexedRingBuffer<T>::getLast(T& res)
{
  bool isnewdata = true;

  pthread_mutex_lock( &buffermutex );
  if (windex == rindex) isnewdata = false;
  res = buffer[windex];
  rindex = windex;
  pthread_mutex_unlock( &buffermutex );

  return isnewdata;
}

template <class T>
bool MutexedRingBuffer<T>::getNext(T& res, unsigned long usec_patience)
{
  int err;
  timespec ts_timeout;
  timeval  tv_now;
  timeval  tv_timeout;
 
  if (usec_patience > 0) {
    gettimeofday( &tv_now, 0);
    tv_timeout.tv_usec = (tv_now.tv_usec + usec_patience) % 1000000;
    tv_timeout.tv_sec  = tv_now.tv_sec + ((tv_now.tv_usec + usec_patience) / 1000000);
    ts_timeout.tv_sec  = tv_timeout.tv_sec;
    ts_timeout.tv_nsec = tv_timeout.tv_usec * 1000;
    
    pthread_mutex_lock( &condition_mutex );
    err = pthread_cond_timedwait( &condition_cond, &condition_mutex, &ts_timeout);
    pthread_mutex_unlock( &condition_mutex );
  }
  return getLast(res);
}

template <class T>
void MutexedRingBuffer<T>::getAll(std::vector< T >& res)
{
  pthread_mutex_lock( &buffermutex );
  res.clear();
  for (unsigned int i=0; i<buffer_size; i++)
    {
      res.push_back( buffer[(windex + i) % buffer_size]);
    }
  rindex = windex;
  pthread_mutex_unlock( &buffermutex );
}

template <class T>
bool MutexedRingBuffer<T>::getNew(std::vector< T >& res)
{
  bool isnewdata = true;  
  pthread_mutex_lock( &buffermutex );
  res.clear();  
  if (rindex == windex) {
    isnewdata = false;
  }
  else {      
    do {
      rindex = (rindex +1) % buffer_size;
      res.push_back( buffer[rindex]);
    } while (rindex != windex);
  }
  pthread_mutex_unlock( &buffermutex );
  
  return isnewdata;
}

}

#endif 
