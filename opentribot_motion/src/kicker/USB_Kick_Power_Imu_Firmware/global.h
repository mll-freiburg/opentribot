#define TIMSK TIMSK1
#include "avrlibdefs.h"
#include "avrlibtypes.h"


#define F_CPU      16000000                 // 7.37MHz processor
//#define F_CPU        4000000                          // 4MHz processor
//#define F_CPU        3686400                          // 3.69MHz processor
#define CYCLES_PER_US ((F_CPU+500000)/1000000)  // cpu cycles per microsecond


