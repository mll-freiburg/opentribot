#ifndef _MY_ERR_MSG_H_
#define _MY_ERR_MSG_H_

#include <iostream>

#ifndef THIS_ERROUT
#define THIS_ERROUT(__msg__)  *this->errout<<  "##ERR  | " << __FILE__ << " | " << __PRETTY_FUNCTION__ << " | " << __msg__ << "\n" << std::flush
#endif

#ifndef THIS_INFOOUT
#define THIS_INFOOUT(__msg__) *this->infoout<< "##INFO | " << __FILE__ << " | " << __PRETTY_FUNCTION__ << " | " << __msg__ << "\n" << std::flush
#endif

#endif
