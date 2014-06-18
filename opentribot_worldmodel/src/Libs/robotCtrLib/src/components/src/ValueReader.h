/*
 * Copyright (c) 1999 - 2001, Artur Merke <amerke@ira.uka.de> 
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _VALUEREADER_H_
#define _VALUEREADER_H_

/**
   this is an improved code for the earlier developed ValueParser
*/

//#include <string>
#include <iostream>
#include "str2val.h"

using namespace std;

/**
 Ein einfacher Zeilenparser, der Eintraege der Form

 <string> = <werte>

 einlesen kann.
 Als <werte> koennen folgende Eintraege vorkommen

 <werte> ::= <int>    | <int>*
 <werte> ::= <long>   | <long>*
 <werte> ::= <float>  | <float>*
 <werte> ::= <double> | <double>*
 <werte> ::= <string>
 <werte> ::= <bool>

 Der Zugriff auf solche Zeilen erfolgt mittels der typisierten Methoden
 get(const char *,int&);  
 get(const char *,long&); ... usw.

 Es finden keine Fehlerkorrekturen der Eingabe statt. Z.B ist 2.1 keine gueltige
 Eingabe fuer ein int!!!

 
 Wir betrachten dazu als Beispiel die Methode 
 get(const char *,int&) mit dem Aufruf
 get("key",i) 
 
 Beispiele fuer Eintraege in der Datei und Interpretationen sind 

 key= 123            -> i= 123
 key= 123  124       -> i= 123              (Warnung)
 key= 3.9            -> i= <undefiniert>    (Warnung)
 key=                -> i= <undefiniert>    (Warnung)
 key= 7a             -> i= <undefiniert>    (Warnung) 
 key= a              -> i= <undefiniert>    (Warnung)
 key= 11111111111111 -> i= <undefiniert>    (Warnung da zu gross)

 Von den Typen int,long,float,double,bool,char koennen auch Arrays eingelesen werden.
 Die uebergebenen Feleder muessen bereist genuegend Speicher fuer alle Eintraege 
 reserviert haben. 
 
 Auch hierzu ein Paar Beispiele fuer die Methode

 get(const char *,int* value, int len) mit dem Aufruf
 get("key",a,2);

 ( Das Feld a muss dazu durch  int a[k]; oder int *a= new int[k]; (k>=2) o.a.
   Speicher reserviert haben. )

 key= 123 124            -> a= [123,124]
 key= 123 124 124        -> a= [123,124]              (Warnung)
 key= 123                -> a= [123,<undefiniert>]    (Warnung)
 key= 11111111111111 123 -> a= [<undefiniert>,<undefiniert>]  (Warnung da erster Wert zu gross)

 key= 3.1 123            -> a= [<undefiniert>, <undefiniert>]  (Warnung) 
                                  ".1 123" wird als ungueltige Eigabe betrachtet

 key=                    -> a= [<undefiniert>, <undefiniert>] (Warnung)


 Zu jeder Methode gibt es auch eine Variante, die einen default Wert setzen kann. 
 Dieser werden als Parameter uebergeben.

 Anmerkungen zur Implementierung:

 Die jetzige Implementierung ist nicht sehr effizient, reicht aber locker fuer
 Datein mit weniger als ein Paar Hundert Eintraegen. Es wird keine Hashtabelle 
 verwendet, und der Aufwand fuer das Einlesen eines Bezeichners ist linear in der
 Anzahl aller Eintraege.

 Autor: Artur Merke
*/

class KeyValueDict {
 public:
  virtual bool insert_key(char const* key, int len= -1)= 0; 
  virtual bool insert_val(char const* val, int len= -1)= 0; 
  virtual bool append_val(char const* val, int len= -1, bool sep= true)= 0;
  virtual int get_last_val_with_key(char const* key, char const* & val)= 0;
  virtual void show(ostream & out) const { }
};

/**
   the class KeyValueTable is a very inefficient implementation of the 
   KeyValueDict concept wrt. to the searching time for an entry!
 
   A better implementation would involve a binary&balanced tree or a hash
   table, but this makes sense if you ever will have to read more then several
   hundred of entries.
*/
class KeyValueTable: public KeyValueDict {
  int max_size;
  int cur_size;
  struct Entry {
    Entry() { access= 0; key= 0; val= 0; val_len= 0;}
    int  access;
    char * key;
    char * val;
    int val_len;
  };
  Entry * tab;

  KeyValueTable(int new_max_size);

  /// i must be between 0 <= i < cur_size
  bool set_key(int i,const char * key, int len= -1);
  bool set_last_key(const char * key, int len= -1) { return set_key(cur_size-1,key,len); }

  /// i must be between 0 <= i < cur_size
  bool set_val(int i,const char * value, int len= -1);
  bool set_last_val(const char * value, int len= -1) { return set_val(cur_size-1,value,len); }


  /** i must be between 0 <= i < cur_size
      \param sep if true, then a separating ' ' is included if existing value is != 0;
  */
  bool append_val(int i,const char * value, int len= -1, bool sep= true);

  ///sets new cur_size, uses set_max_size if necessary
  bool set_cur_size(int size);
  bool inc_cur_size() { return set_cur_size(cur_size+1); }

  ///enlarges the table, cur_size remains the same (incl. the entries)
  bool set_max_size(int size);
 public:
  KeyValueTable();
  virtual ~KeyValueTable();

  bool insert_key(char const* key, int len= -1) {
    inc_cur_size();
    return set_last_key(key,len);
  }

  bool insert_val(char const* val, int len= -1) {
    return set_last_val(val,len);
  }

  bool append_val(char const* val, int len= -1, bool sep= true) {
    return append_val(cur_size-1,val, len,sep);
  }

  //bool get_first_val_with_key(char const* key, char const* & val);

  //returns the length of val 
  int get_last_val_with_key(char const* key, char const* & val);

  void show(ostream & out) const;
};

class ValueReader {
  friend ostream& operator<< (ostream&,const ValueReader &);
protected:
  // Members
  KeyValueDict * kvd;

  int max_key_length; //just for better output
public:
  bool verbose_mode;
  char assignment_char;
  char comment_char;

  ValueReader();
  ~ValueReader() { if (kvd) delete kvd; }

  bool append_from_command_line(int argc, char const* const* argv, char const* prefix_str= 0);
  bool append_from_string(char const* str);
  bool append_from_file(char const* filename, char const* block);

  void show(ostream & out) const { kvd->show(out); }

  int get(const char* key, char const* & val) {
    int res= kvd->get_last_val_with_key(key,val);
    if (verbose_mode)
      cout << "\n" << key << "= (" << res << ") " << val; 
    return res;
  }
  
  //specific versions
  int get(char const* key, char & val, char const* & next);
  int get(char const* key, char * val, int num, char const* & next);

  //generic versions
  template<class TTT> int get(char const* key, TTT & val, char const* & next);

  template<class TTT> int get(char const* key, TTT & val);

  template<class TTT> int get(char const* key, TTT * val, int num, char const* & next);
  template<class TTT> int get(char const* key, TTT * val, int num);
};

template<class TTT> 
int ValueReader::get(char const* key, TTT & val, char const* & next) {
  int res= get(key,next);
  if ( res < 0 )
    return res;
  
  res= str2val(next,val,next);
  strspace(next,next);
  return res;
}

template<class TTT> 
int ValueReader::get(char const* key, TTT & val) {
  char const* next;
  return get(key,val,next);
}

template<class TTT> 
int ValueReader::get(char const* key, TTT * val, int num, char const* & next) {
#if 0
  int res= get(key,next);
  if ( res < 0 )
    return res;

  res= str2val(next,num,val,next);
  strspace(next,next);
  return res;
#else
  int res= get(key,next);
  if ( res < 0 )
    return res;

  int size=0;
  if (num < 1) {
    return 0;
  }

  while (size < num) {
    const char * dum;
    int res= str2val(next, *val, dum);
    if (res) {
      val++;
      size++;
      next= dum;
    }
    else
      break;
  }
  strspace(next,next);
  return size;
#endif
}

template<class TTT> 
int ValueReader::get(char const* key, TTT * val, int num) {
  char const* next;
  return get(key,val,num,next);
}

#endif
