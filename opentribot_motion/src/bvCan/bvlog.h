#ifndef BV_LOG
#define BV_LOG

#include <cstdlib>
#include <iostream>

// a fancy way to write to cerr which leaves us the possibility to
// introduce more advanced logging later
/*
struct BV_Log : public std::basic_ostream<char, std::char_traits<char> > {
 
        BV_Log() : std::basic_ostream<char, std::char_traits<char> >( std::cerr.rdbuf() ) {}
 
};
*/

struct BV_Log : public std::ostream {
 
        BV_Log() : std::ostream( std::cerr.rdbuf() ) {}
 
};

// global_Log should be constructed somewhere else, look at bvglobal.cpp
extern BV_Log global_Log;

#endif

