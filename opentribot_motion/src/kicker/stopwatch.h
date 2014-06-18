#ifndef STOPWATCH
#define STOPWATCH
#include <sys/time.h>


struct StopWatch
{
 int alarm_usecs;
 struct timeval tv1, tv2;
   StopWatch ()
 {
   reset ();
 }
 int reset ()
 {
   gettimeofday (&tv1, NULL);
   return 0;
 }
 int get_usecs ()
 {

   gettimeofday (&tv2, NULL);
   int secs, usecs;
   secs = tv2.tv_sec - tv1.tv_sec;
   usecs = 1000 * 1000 * secs + tv2.tv_usec - tv1.tv_usec;
   return usecs;
 }
 int get_msecs ()
 {
   return get_usecs () / 1000;
 }
 int set_alarm(int usecs){
  alarm_usecs=usecs;
  return 0;
 }
 bool sleep_to_alarm();

};

#endif
