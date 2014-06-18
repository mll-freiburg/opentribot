#ifndef BV_DEBUG_H
#define BV_DEBUG_H

#include "bvlog.h" 

/**
  use this file to send debug information to the log device
*/

/**
    location of error and normal output stream
*/
#define BV_CERR global_Log
#define BV_COUT global_Log

#ifndef BV_DBG_LOCATION
#define BV_DBG_LOCATION "["__FILE__ " L" << __LINE__ << "]: "
#endif

#define BV_DEBUG(CODE) CODE


/**
========================================================================
  * predefined VERBOSITY levels 
========================================================================
*/
 
#define BV_LFATAL       1  /* absolutely nothing, apart from fatal err. */
#define BV_LSEVERE      10  /* severe errors (inconsistencies,
                                    no memory,... */
#define BV_LWARNING     20  /* possible problems, like out of
                                    bounds access, div. by zero, etc...     */
#define BV_LINFO        30  /* important protocol information,
                                    internally calculated parameters, used
                                    current parameter values, etc... */
#define BV_LDETAILLED   40  /* + more, possible hints for improvements.. */
 
#define BV_LDEBUGINFO   500  
#define BV_LDEBUGINFO1  600
#define BV_LDEBUGINFO2  700  
#define BV_LDEBUGINFO3  800 
#define BV_LDEBUGINFO4  900  
#define BV_LDEBUGINFO5  1000  

/**
========================================================================
  * the following are the default values for the 
  * BV_VERBOSITY in the cases where:
  *   NDEBUG is DEFINED or NOT  
========================================================================
*/

#ifndef BV_VERBOSITY
#ifndef NDEBUG
#define BV_VERBOSITY BV_LINFO
#else
#define BV_VERBOSITY BV_LSEVERE
#endif
#endif

#ifndef  BV_SAFETY_LEVEL
#define  BV_SAFETY_LEVEL    BV_SAFETY
#endif


// define the BV_VERBOSITY_LEVEL as already defined BV_VERBOSITY
#ifndef  BV_VERBOSITY_LEVEL
#define  BV_VERBOSITY_LEVEL BV_VERBOSITY
#endif

/**
========================================================================
  * define levels of debugging 
========================================================================
*/
 
#define BV_FATAL(STRING)     BV_CERR << "# FATAL " BV_DBG_LOCATION << STRING << std::endl
#define BV_FATALCOL(STRING)  BV_CERR << "# FATAL " BV_DBG_LOCATION << STRING << std::endl ,

#undef  BV_SEVERE 
#if BV_VERBOSITY_LEVEL < BV_LSEVERE 
#define BV_SEVERE(STRING)      
#else  
#define BV_SEVERE(STRING)    BV_CERR << "# SEVERE " BV_DBG_LOCATION << STRING << std::endl
#endif  

#undef BV_WARNING
#undef BV_WARNINGNNL
#undef BV_WARNINGC
#undef BV_WARNCOL
#if BV_VERBOSITY_LEVEL < BV_LWARNING
#define BV_WARNING(STRING)
#define BV_WARNINGNNL(STRING)
#define BV_WARNINGC(STRING) 
#define BV_WARNCOL(STRING)
#else
#define BV_WARNING(STRING)      BV_CERR << "# WARNING " BV_DBG_LOCATION << STRING << std::endl
#define BV_WARNINGNNL(STRING)   BV_CERR << "# WARNING " BV_DBG_LOCATION << STRING 
#define BV_WARNINGC(STRING)     BV_CERR << STRING
#define BV_WARNCOL(STRING)      BV_CERR << "# WARNING " BV_DBG_LOCATION << STRING << std::endl , 
#endif  


#undef BV_INFO
#undef BV_INFONNL
#undef BV_INFOC
#if BV_VERBOSITY_LEVEL < BV_INFO
#define BV_INFO(STRING)
#define BV_INFONNL(STRING)
#define BV_INFOC(STRING)
#else
#define BV_INFO(STRING)    BV_COUT << "# Debug Info " BV_DBG_LOCATION << STRING << std::endl
#define BV_INFONNL(STRING) BV_COUT << "# Debug Info " BV_DBG_LOCATION << STRING 
#define BV_INFOC(STRING)   BV_COUT << STRING 
#endif 


#undef BV_DETAILLED
#if BV_VERBOSITY_LEVEL < BV_LDETAILLED
#define BV_DETAILLED(STRING)
#else
#define BV_DETAILLED(STRING) BV_COUT << "# Det. Info " BV_DBG_LOCATION << STRING << std::endl
#endif

#undef BV_DEBUGINFO
#undef BV_DEBUGINFONNL
#undef BV_DEBUGINFOC
#if BV_VERBOSITY_LEVEL < BV_LDEBUGINFO
#define BV_DEBUGINFO(STRING)
#define BV_DEBUGINFONNL(STRING)
#define BV_DEBUGINFOC(STRING)
#else
#define BV_DEBUGINFO(STRING)    BV_COUT << "# Debug Info " BV_DBG_LOCATION << STRING << std::endl
#define BV_DEBUGINFONNL(STRING) BV_COUT << "# Debug Info " BV_DBG_LOCATION << STRING 
#define BV_DEBUGINFOC(STRING)   BV_COUT << STRING 
#endif 

#undef BV_DEBUGINFO1
#if BV_VERBOSITY_LEVEL < BV_LDEBUGINFO1
#define BV_DEBUGINFO1(STRING)
#else
#define BV_DEBUGINFO1(STRING) BV_COUT << "# Debug Info01 " BV_DBG_LOCATION << STRING << std::endl
#endif

#undef BV_DEBUGINFO2
#if BV_VERBOSITY_LEVEL < BV_LDEBUGINFO2
#define BV_DEBUGINFO2(STRING)
#else
#define BV_DEBUGINFO2(STRING) BV_COUT << "# Debug Info02 " BV_DBG_LOCATION << STRING << std::endl
#endif

#undef BV_DEBUGINFO4
#undef BV_DEBUGINFO4NNL
#undef BV_DEBUGINFO4C
#if BV_VERBOSITY_LEVEL < BV_LDEBUGINFO4
#define BV_DEBUGINFO4(STRING)
#define BV_DEBUGINFO4NNL(STRING)
#define BV_DEBUGINFO4C(STRING)
#else
#define BV_DEBUGINFO4(STRING)    BV_COUT << "# Debug Info4 " BV_DBG_LOCATION << STRING << std::endl 
#define BV_DEBUGINFO4NNL(STRING) BV_COUT << "# Debug Info4 " BV_DBG_LOCATION << STRING 
#define BV_DEBUGINFO4C(STRING)   BV_COUT << STRING 
#endif 



#undef BV_DEBUGINFO5
#undef BV_DEBUGINFO5NNL
#undef BV_DEBUGINFO5C
#if BV_VERBOSITY_LEVEL < BV_LDEBUGINFO5
#define BV_DEBUGINFO5(STRING)
#define BV_DEBUGINFO5NNL(STRING)
#define BV_DEBUGINFO5C(STRING)
#else
#define BV_DEBUGINFO5(STRING)    BV_COUT << "# Debug Info5 " BV_DBG_LOCATION << STRING << std::endl
#define BV_DEBUGINFO5NNL(STRING) BV_COUT << "# Debug Info5 " BV_DBG_LOCATION << STRING 
#define BV_DEBUGINFO5C(STRING)   BV_COUT << STRING 
#endif 

/**
========================================================================
*/

#define _BV_ABORT            std::abort()  

#define BV_VERIFY(EXPRESSION)\
      ((void)((EXPRESSION)?  \
               1 :           \
               (BV_WARNCOL(#EXPRESSION " evaluated to false") 0)))

#define BV_ASSERT(EXPRESSION)\
      ((void)((EXPRESSION)? 1 : \
              (BV_FATALCOL( "assertion " #EXPRESSION " failed")\
               _BV_ABORT , 0) ))

#endif
