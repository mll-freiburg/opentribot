
#include "Journal.h"
#include "../Fundamental/Time.h"
#include <cmath>
#include <sstream>
#include <stdlib.h>

using namespace Tribots;
using namespace std;

Journal Journal::the_journal;

namespace {

  std::string toDotNot (const Time& t) {
    unsigned long int sec = t.get_msec()/1000;
    unsigned long int msec = t.get_msec()%1000;
    stringstream inout;
    inout << sec << '.';
    if (msec<10)
      inout << "00";
    else if (msec<100)
      inout << '0';
    inout << msec << endl;
    std::string dest;
    getline (inout, dest);
    return dest;
  }

}

Journal::Journal () throw () : verbosity (1), output_stream_pointer (NULL), output_file_pointer (NULL) {;}

Journal::~Journal () throw () {
  flush ();
  if (output_file_pointer)
    delete output_file_pointer;
}

void Journal::flush () throw () {
  if (output_stream_pointer)
    output_stream_pointer->flush();
  if (!message_buffer.empty()) {
    if (flush_output.length()==0)
      write_buffer (std::cerr);
    else {
      ofstream f (flush_output.c_str());
      if (f)
        write_buffer (f);
      else
        write_buffer (std::cerr);
    }
  }
}  

void Journal::set_mode (const ConfigReader& vread) throw (Tribots::InvalidConfigurationException) {
  int new_verbosity;
  bool rotate_log = true;
  vread.get("rotate_log",rotate_log);
  if (vread.get ("Journal::verbosity", new_verbosity)>0)
    verbosity=new_verbosity;
  string confline;
  if (vread.get ("Journal::output", confline)<=0)
    throw InvalidConfigurationException ("Journal::output");
  if (confline == "Journal::memory")
    set_memory_mode();
  else if (confline == "stdout")
    set_stream_mode (cout);
  else if (confline == "stderr")
    set_stream_mode (cerr);
  else {
    // create time-dependent output file (e.g. journal.out.2781289)
    stringstream filename;
    filename << confline;
    
    if (rotate_log) {
      struct timeval tv1;
      Time ttstart;
      ttstart.set_usec(0);
      ttstart.get (tv1);
      
      struct tm* mytm;
      mytm = localtime(&tv1.tv_sec);
      stringstream timestring;
      timestring << (1900+mytm->tm_year) << "-";
      if (mytm->tm_mon < 9) timestring << "0";
      timestring << mytm->tm_mon+1 << "-";
      if (mytm->tm_mday < 10) timestring << "0";
      timestring << mytm->tm_mday << "-";
      if (mytm->tm_hour < 10) timestring << "0";
      timestring << mytm->tm_hour;
      if (mytm->tm_min < 10) timestring << "0";
      timestring << mytm->tm_min;
      if (mytm->tm_sec < 10) timestring << "0";
      timestring << mytm->tm_sec;
      
      filename << "." << timestring.str();
    }
    
    ofstream* errfile = new ofstream (filename.str().c_str());
    if (!(*errfile)) {
      cerr << "Fehler beim Oeffnen der Protokolldatei " << confline << "\n";
      delete errfile;
      throw InvalidConfigurationException ("Journal::output");
    }
    if (rotate_log) {
      // delete existing link or file journal.out
      stringstream deletecmd;
      deletecmd << "rm -f " << confline << " >/dev/null 2>&1";
      system(deletecmd.str().c_str());
      // update link to journal.out
      stringstream relinkcmd;
      relinkcmd << "ln -s " << filename.str() << " " << confline;
      system(relinkcmd.str().c_str());
    }
    if (output_stream_pointer)
      output_stream_pointer->flush();
    if (output_file_pointer)
      delete output_file_pointer;
    output_stream_pointer=errfile;
    output_file_pointer=errfile;
  }
  if (vread.get ("Journal::flush", confline)<=0)
    flush_output = "";
  else
    flush_output = confline;
}

void Journal::set_verbosity (int i) throw () {
  verbosity=i;
}

void Journal::set_memory_mode () throw () {
  if (output_stream_pointer)
    output_stream_pointer->flush();
  output_stream_pointer = NULL;
  if (output_file_pointer) {
    delete output_file_pointer;
    output_file_pointer=NULL;
  }
}

void Journal::set_stream_mode (ostream& os) throw () {
  if (output_stream_pointer)
    output_stream_pointer->flush();
  output_stream_pointer = &os;
  if (output_file_pointer) {
    delete output_file_pointer;
    output_file_pointer=NULL;
  }
}

void Journal::clear_buffer () throw () {
  while (!message_buffer.empty())
    message_buffer.pop();
}

void Journal::write_buffer (ostream& os) throw () {
  while (!message_buffer.empty()) {
    os << message_buffer.front() << endl;
    message_buffer.pop();
  }
}

namespace {
  // Hilfsfunktion, um aus einem unsigned int einen string zu berechnen
  inline string uint2str (unsigned int iii) {
     if (iii==0)
       return string("0");
     const unsigned int slen = static_cast<unsigned int>(floor(log(static_cast<double>(iii))/log(10.0))+1);
     string res (slen,'0');
     for (int j=slen-1; j>=0; j--) {
       res[j]=static_cast<char>(iii%10+'0');
       iii=iii/10;
     }
     return res;
  }
}

void Journal::error (const char* fname, unsigned int lnum, const char* errstr) throw () {
  Time ctime;
  if (output_stream_pointer)
    (*output_stream_pointer) << toDotNot (ctime) << " Error in " << fname <<  ", " << lnum << ": " << errstr << endl;
  else 
    try{
      message_buffer.push(uint2str(ctime.get_msec()/1000)+string(".")+uint2str(ctime.get_msec()%1000)+string(" Error in ")+string(fname)+string(", ")+uint2str(lnum)+string(": ")+string(errstr));
    }catch(bad_alloc&){
      // bei Problemen gebe den Fehler auf der Konsole aus
      cerr << "Speicherueberlauf in " << __FILE__ << ", " << __LINE__ << "\n";
      cerr << ctime << " Error in " << fname <<  ", " << lnum << ": " << errstr << "\n";
    }
}

void Journal::warning (const char* fname, unsigned int lnum, const char* errstr) throw () {
  if (verbosity>=2) {
    Time ctime;
    if (output_stream_pointer)
      (*output_stream_pointer) << toDotNot (ctime) << " Warning in " << fname <<  ", " << lnum << ": " << errstr << endl;
    else 
      try{
        message_buffer.push(uint2str(ctime.get_msec()/1000)+string(".")+uint2str(ctime.get_msec()%1000)+string(" Warning in ")+string(fname)+string(", ")+uint2str(lnum)+string(": ")+string(errstr));
      }catch(bad_alloc&){
        // bei Problemen gebe den Fehler auf der Konsole aus
        cerr << "Speicherueberlauf in " << __FILE__ << ", " << __LINE__ << "\n";
        cerr << ctime << " Warning in " << fname <<  ", " << lnum << ": " << errstr << "\n";
      }
  }
}

void Journal::message (const char* errstr, bool timestamp) throw () {
  if (verbosity>=3) {
    Time ctime;
    if (output_stream_pointer) {
      if (timestamp) {
        (*output_stream_pointer) << toDotNot (ctime) << ": " << errstr << endl;
      } else {
        (*output_stream_pointer) << errstr << endl;
      }
    } else {
      try{
        if (timestamp) {
          message_buffer.push(uint2str(ctime.get_msec()/1000)+string(".")+uint2str(ctime.get_msec()%1000)+string(errstr));
        } else {
          message_buffer.push(string(errstr));
        }
      }catch(bad_alloc&){
        // bei Problemen gebe den Fehler auf der Konsole aus
        cerr << "Speicherueberlauf in __FILE__, __LINE__\n";
        cerr << ctime << ": " << errstr << "\n";
      }
    }
  }
}


void Journal::sound_message (const char* outstr) throw () {
  
      
FILE   *outFile;
outFile=fopen("/tmp/ausgabe.txt","w");
fprintf(outFile,outstr);
fclose(outFile);
      
      

}





