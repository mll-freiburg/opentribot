#include "bvfd.h"

/** ctor */
BVfd::BVfd(){
  // in this class I will not request a value for the fd
  // from operating system
  // so , simply I set fd to -1, so till when that a legal value is put 
  // in fd, I can check fd directly for validity
  // because, fd=-1 is a nonvalid file descriptor
  setfd(-1);
  BV_ASSERT( getrfd() == getwfd() );
  BV_DEBUGINFO5( this << " BVfd::ctor fd=" << getrfd() );
}

/** ctor
    @param, a fiel descriptor
*/
BVfd::BVfd(int afd){
  setfd(afd);
  BV_ASSERT( getrfd() == getwfd() );
  BV_DEBUGINFO5( this << " BVfd::ctor fd=" <<  getrfd() );
}


/** dtor */
BVfd::~BVfd(){
  BV_ASSERT( getrfd() == getwfd() );
  BV_DEBUGINFO5( this << " BVfd::dtor fd=" << getrfd() );
}

/** set the file descriptor */
void BVfd::setfd(int afd){
  setrfd(afd);
  setwfd(afd);
  BV_ASSERT( getrfd() == getwfd() );
  BV_DEBUGINFO5( this << " BVfd::setfd fd=" << getrfd() );
}
 
/** get the file descriptor */
int BVfd::getfd() const {
  BV_ASSERT( getrfd() == getwfd() );
  return getrfd();
}
 
/** @return true if fd blocks, false otherwise
    -1, on error
*/
int BVfd::is_blocking() 
{
  BV_ASSERT( getrfd() == getwfd() );
  int fd=getrfd();
  // no way to fail
  long flags = fcntl( fd, F_GETFL );
  if(flags==-1){
    BV_WARNING("BVfd::is_blocking error on calling fcntl, -1 is returned by fcntl");
    // FIXME : what should I do now !!!!!!!!!!!!!!!!!!!
    return flags; 
  }
  return ! (flags & O_NONBLOCK);
}

/** toggle_blocking() could be const, but conceptionally it isn't */
void BVfd::toggle_blocking()
{
  BV_ASSERT( getrfd() == getwfd() );
  int fd=getrfd();

  long flags = fcntl( fd, F_GETFL );
  //toggle non-blocking flag
  flags ^= O_NONBLOCK;
  if ( fcntl( fd, F_SETFL, flags ) )
    BV_WARNING( "BVfd::toggle_blocking toggle blocking failed (fcntl) " );
}

/** waits a certain amount of time for the discriptor to change status and
    the functionality (readable/writeable) to become available
    @param  maximum interval before we return, in sec
    @return true if descriptor 'function' is (now) available
*/
bool BVfd::wait( int what, double max_wait ){



  fd_set   *r = NULL;
  fd_set   *w = NULL;
  fd_set    rfs, wfs;

  BV_ASSERT( getrfd() == getwfd() );
  int fd=getrfd();


  if ( what != readable && what != writable ){
    BV_WARNING(this <<  " argument must be 'readable' or 'writeable'" );
    return false;
  }

  if ( what == readable ){
    r = &rfs;
    FD_ZERO(r);
    FD_SET (fd,r);
  }
  if ( what == writable ){
    w = &wfs;
    FD_ZERO(w);
    FD_SET (fd,w);
  }

  max_wait   = max_wait > 0 ? max_wait : 0;
  long  secs = (long) max_wait;
  struct timeval tv;
  tv.tv_sec  = secs;
  tv.tv_usec = (long) (1e6 * (max_wait - secs));

  int result = select(fd+1, r, w, NULL, &tv);
  
  if ( result > 0 ){  // no of changed descriptors
  
    // did descriptor become available for the selected functionality?
    //PT_SAFE(
    if ( what == readable )
      BV_ASSERT( FD_ISSET(fd,r) );
    if ( what == writable )
      BV_ASSERT( FD_ISSET(fd,w) );
    //)
    return true;
  }
  else {
    // either descriptor hasn't changed or there was another problem
    if ( result < 0 ){
    BV_WARNING (this <<  " select failed " );
    }
    return false;
  }
}

/** write count bytes to the file descriptor
    this function works on the basis of
 
    ssize_t write(int fd, const void *buf, size_t count);
      
    it actually calls the above fucntion.
    So, it will handle the case when the fd is a non valid discriptor
  
    @return , the return value of the
    ssize_t write(int fd, const void *buf, size_t count);
*/
ssize_t BVfd::write(const void *buffer, size_t count){
  BV_ASSERT( getrfd() == getwfd() );
  int fd=getrfd();

  if ( count <=0 ){
    BV_WARNING( "BVfd::write attempts to write/sent " << count << " bytes" );
  }

  // in the case that file descriptor is not valid, send a WARNING
  if( fd == -1 ){
    BV_WARNING( "BVfd::write fd=[" << fd << "] is not valid " );	  
  }
  
  // do some debugging      	
  for(unsigned int i=0; i < count; i++)
    // note that I should cast *( (char*)buffer + i ) to int
    // also I should and it with 0xff
    BV_DEBUGINFO5( "BVfd::write buffer[" << i << "]=0x[" << hex << ((int)(*( (char*)buffer + i )) & 0xff)<< "]" << dec);
  
  ssize_t retval;
  retval = ::write(fd,buffer,count);

  // On success, the number of bytes written are returned (zero indicates nothing was written).
  // On error, -1 is returned, and errno is set appropriately.
  // If count is zero and the file descriptor refers to a regular file, 0 will be returned
  // without causing any other effect.
  // For a special file, the results are not portable.
	  
  if ( retval < 0 ){
    BV_WARNING( "BVfd::write write error" );
    int errnoold = errno;
    errorLog(errnoold);
  }
  else if ( retval ==0 ) BV_WARNING( "BVfd::write nothing was written" );
  else if ( retval > 0 ) BV_DEBUGINFO5( "BVfd::write wrote " << retval << " bytes successfully");
  
  return retval;
}

/** read "count" bytes from the file descriptor
    this function works on the basis of
 
    ssize_t read(int fd, void *buf, size_t count);
 
    it actually calls the above function
    So, it will handle the case when the fd is a non valid discriptor
 
    @return , the return value of the
    ssize_t read(int fd, void *buf, size_t count);
*/
ssize_t BVfd::read( void *buffer, size_t count){

  BV_ASSERT( getrfd() == getwfd() );
  int fd=getrfd();
	
  if ( count <=0 ){
    BV_WARNING(this <<  "BVfd::read fd=[" << fd << "] attempt to read/revc " << count << " bytes" );
  }
  // in teh case that file descriptor is not valid, send a WARNING
  if( fd == -1 ){
    BV_WARNING( "BVfd::read fd=[" << fd << "] is not valid " );	  
  }
  
  ssize_t retval;
  retval = ::read(fd,buffer,count);
  // On success, the number of bytes read is returned (zero indicates end of file),
  // and the file position is advanced by  this number.
  // It is not an error if this number is smaller than the number of bytes requested;
  // this may happen for example because fewer bytes are actually available right now
  // (maybe because we were close to  end-of-file, or because we are reading from a pipe,
  //  or from a terminal), or because read() was interrupted by a signal.
  //  On error, -1 is returned, and errno is set appropriately.
  //  In this case it is left unspecified whether the file position (if any) changes.
  
  if ( retval < 0 ){
    BV_WARNING( "  BVfd::read read error read from fd=" << fd << " returns [" << retval << "]" );
    if(retval==EOF) BV_WARNING("BVfd::read read returns EOF");
    int errnoold = errno;
    errorLog(errnoold);
  }
  else if ( retval ==0 ) BV_DEBUGINFO5( "BVfd::read read eof" );
  else if ( retval > 0 ){
    BV_DEBUGINFO5( "BVfd::read read " << retval << " byte" );
    for(int i=0; i<retval; i++)
      // note that I should cast *( (char*)buffer + i ) to int
      // also I should and it with 0xff
      BV_DEBUGINFO5("BVfd::read buffer[" << i << "]=0x[" << hex << ((int)(*( (char*)buffer + i)) & 0xff) << "]" << dec);
  }  
  return retval;
}

int BVfd::trywait( int param ) {}

int BVfd::read(void* param) {
  std::string::size_type  stringLength;
  ssize_t countLength=read( (char *)&stringLength,sizeof(stringLength));
  //char bufString[1000];
  char bufString[stringLength * sizeof(char) ];
  ssize_t countString=read(bufString,stringLength);
  
  stringstream ss;
  for(unsigned int i=0; i< countString ;i++){
    //BV_DEBUGINFO4("BVfd::read bufString[" << i << "]=" << (int)bufString[i]);
    ss < (unsigned char)bufString[i];
  }
  *((string*)param) = ss.str();
  return (countLength+countString); 
}

int BVfd::write(void* param) {
  string localString=  *((string*)param);
  stringstream ss;
  ss < localString;
  // get the string from ss
  string strToSend = ss.str();
  return write(strToSend.c_str(),strToSend.size());
}

/** write to log device a string based on errno
    this function will be called by read and write when the
    return value of ::read and ::write is less than zero
*/
void BVfd::errorLog(int errnoold){
  string strError( strerror(errnoold) );
  string strErrMan;

  switch(errnoold){
  case EINTR:
    strErrMan = "The call was interrupted by a signal before any data was read or written";
    break;
  case EAGAIN:
    strErrMan = "Non-blocking I/O has been selected using O_NONBLOCK, no data available for reading and write would block";
    break;
  case EIO:
    strErrMan = "I/O error";
    break;
  case EISDIR:
    strErrMan = "fd refers to a directory";
    break;
  case EBADF:
    strErrMan = "fd is not a valid file descriptor or is not open for reading/writing";
    break;
  case EINVAL:
    strErrMan = "fd is attached to an object which is unsuitable for reading/writing";
    break;
  case EFAULT:
    strErrMan = "buf is outside your accessible address space";
    break;
  case EPIPE:
    strErrMan = "fd  is  connected  to a pipe or socket whose reading end is closed";
    break;
  case ENOSPC:
    strErrMan = "The device containing the file referred to by fd has no room for the data";
    break;
//case HOST_NOT_FOUND:   // normally 1
//  BV_WARNING( "  BVInetSocket::connect host not found" );
//  break;
  default:
    strErrMan = "unknown error";
    break;
  }
    BV_WARNING("BVfd::errorLog errno=[" << errnoold << "]  from man page : [" << strErrMan  << "]");
    BV_WARNING("BVfd::errorLog errno=[" << errnoold << "]  [" << strError << "]");
}
  

