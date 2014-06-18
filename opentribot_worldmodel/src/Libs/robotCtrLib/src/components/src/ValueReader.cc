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

#include "ValueReader.h"
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <cmath>


KeyValueTable::KeyValueTable() {
  cur_size= 0;
  max_size= 0;
  tab= 0;
}

KeyValueTable::KeyValueTable(int new_max_size) {
  cur_size= 0;
  max_size= 0;
  tab= 0;
  set_max_size(new_max_size);
}

KeyValueTable::~KeyValueTable() {
  if (!tab)
    return;
  for (int i=0; i<cur_size;i++) {
    if (tab[i].key)
      delete[] tab[i].key;
    if (tab[i].val)
      delete[] tab[i].val;
  }
  delete[] tab;
}

bool KeyValueTable::set_cur_size(int size) {
  if (size < 0)
    return false;

  if (size <= max_size) 
    cur_size= size;
  else {
    if (size/2 > max_size)
      set_max_size(size+10);
    else
      set_max_size(2*size+10);
    cur_size= size;
  }
  return true;
}

bool KeyValueTable::set_key(int i, const char * value, int len) {
  if (i < 0 || i >= cur_size) {
    cerr << "\nKeyValueTable::set_key: index " << i << " not in range [0," << cur_size << ")";
    return false;
  }

  if (tab[i].key) {
    delete[] tab[i].key;
    tab[i].key= 0;
  }

  if (!value || len== 0)
    return true;

  if (len < 0)
    len= strlen(value);

  tab[i].key= new char[len+1];
  strncpy(tab[i].key,value,len);
  tab[i].key[len]= '\0';
  return true;
}

bool KeyValueTable::set_val(int i, const char * value, int len) {
  if (i < 0 || i >= cur_size) {
    cerr << "\nKeyValueTable::set_val: index " << i << " not in range [0," << cur_size << ")";
    return false;
  }

  Entry & entry= tab[i];

  if (entry.val) {
    delete[] entry.val;
    entry.val= 0;
  }

  if (!value || len == 0)
    return true;
  
  if (len < 0) 
    len= strlen(value);

  entry.val= new char[len+1];
  strncpy(entry.val,value,len);
  entry.val[len]= '\0';
  entry.val_len= len;
  return true;
}

bool KeyValueTable::append_val(int i, const char * value, int len, bool sep) {
  if (i < 0 || i >= cur_size) {
    cerr << "\nKeyValueTable::append: index " << i << " not in range [0," << cur_size << ")";
    return false;
  }
  if (!value) 
    return true;

  Entry & entry= tab[i];

  if ( len < 0)
    len= strlen(value);

  if ( len <= 0)
    return true;

  char * dum;

  int size= entry.val_len+len+1;
  //cout << "\nsize= " << size << " entry.val_len= " << entry.val_len << " value_len= " << len << " val= " << value;
  if (sep && entry.val_len>0)
    size++;

  dum= new char[size];
  if (entry.val_len>0) {
    strncpy(dum,entry.val,entry.val_len);
    if (sep) {
      dum[entry.val_len]= ' ';
      entry.val_len++; 
    }
  }
  
  strcpy(dum+entry.val_len,value);

  if (entry.val)
    delete[] entry.val;

  entry.val= dum;
  entry.val_len= size-1;
  return true;
}

bool KeyValueTable::set_max_size(int new_max_size) {
  if (new_max_size<0) return false;
  if (new_max_size<=max_size) return true;
  if (cur_size > max_size)
    cur_size= max_size;
  max_size= new_max_size;
  if (cur_size<= 0 || tab==0) {
    if (tab) 
      delete[] tab;
    if (0==max_size) 
      tab= 0;
    else {
      tab= new Entry[max_size];
    }
  } else {
    Entry * dum= tab;
    tab= new Entry[max_size];
    memcpy( (void*)tab,(void*)dum, sizeof(Entry) * cur_size);
    delete[] dum;
  }
  return true;
}

int KeyValueTable::get_last_val_with_key(char const* key, char const* & val) {
  if ( ! key)
    return -1;

  Entry * entry= tab+ cur_size;
  while (entry > tab) {
    entry--;
    //cout << "\nentry.key= " << entry->key << " key= " << key;
    if ( key[0] != entry->key[0] )
      continue;

    if ( strcmp(key,entry->key) != 0 )
      continue;

    val= entry->val;
    return entry->val_len;
  }
  return -1;
}

void KeyValueTable::show(ostream & out) const {
  out << "\ncur_size= " << cur_size << ", max_size= " << max_size;
  for (int i= 0; i< cur_size; i++) 
    out << "\n" << tab[i].key << "= " << "(" << tab[i].val_len << ") " << tab[i].val;
}

#if 0
ostream& operator<< (ostream& o,const KeyValueTable& t) {
  o << "\ncur_size= " << t.cur_size << ", max_size= " << t.max_size;
  for (int i= 0; i<t.cur_size; i++) 
    o << "\n" << t.tab[i].key << "= " << t.tab[i].val;
  return o;
}
#endif
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
ostream& operator<< (ostream& o,const ValueReader & t) {
  o << "\n<ValueReader>";
  t.show(o);
  o << "\n</ValueReader>";
  return o;
}

ValueReader::ValueReader()
  : max_key_length(0)
{
  //verbose_mode= true; //temporal setting
  verbose_mode= false; //temporal setting
  assignment_char= '=';
  comment_char= '#';

  kvd= new KeyValueTable();
}

bool ValueReader::append_from_command_line(int argc, char const* const* argv, char const* prefix_str) {
  int prefix_str_len= 0;
  bool valid_prefix= false;

  if (prefix_str)
    prefix_str_len= strlen(prefix_str);

  if (0 >= argc) 
    return true;

  for (int i=0; i< argc; i++)
  {
    // Free entries left?
    char const* option_str= argv[i];
    if ( '-' == option_str[0] &&  int( option_str[1] ) >= 58 )
    { // must not begin with a ciffer
      if ( prefix_str_len && strncmp(option_str+1,prefix_str,prefix_str_len ) != 0 )
      	valid_prefix= false;
      else
      {
       	valid_prefix= true;
       	kvd->insert_key(option_str + 1 + prefix_str_len);
      }
    }
    else if (valid_prefix) 
      kvd->append_val(option_str);
  }
  return true;
  //cerr << "->found " << numEntries << " entries.";
}

bool ValueReader::append_from_string(char const* line) {
  const char * act=line;
  while ( true ) {
    while( isspace(*act) )
      act++;

    if (*act=='\0' || *act==comment_char) 
      return true; // OK, comment or end

    if (*act== assignment_char) {
      cerr << "\nValueParser: wrong entry line:" << line;
      return false;
    }

    const char * dum= act;

    while ( !isspace(*dum) && *dum!= '\0' && *dum!= assignment_char && *dum!= comment_char)
      dum++;

    if (*dum== '\0' || *dum== comment_char ){
      cerr << "\nValueParser: wrong entry line:" << line;
      return false;
    }

    kvd->insert_key(act,dum-act); // located keyword

    while ( isspace(*dum) )
      dum++;

    if ( *dum != assignment_char ) {
      cerr << "\nValueParser: wrong entry line:" << line;
      return false;
    }

    dum++;

    while ( isspace(*dum) )
      dum++;

    act= dum;
    /* search for the end of the value entry. this end is defined
       as the end of the last succeding string, which is not a key */

    while ( *dum != '\0' && *dum != comment_char && *dum != assignment_char )
      dum++;

    if ( *dum == '\0' || *dum == comment_char) {
      dum--;
      while ( isspace(*dum) )
	dum--;
      dum++;
    }
    else { //go back until you are before the corresponding key
      dum--;
      while ( isspace(*dum) )
	dum--;

      
      if (*dum == assignment_char){
	cerr << "\nValueParser: wrong entry line:" << line;
	return false;
      }
      
      while ( ! isspace(*dum) )
	dum--;

      while ( isspace(*dum) )
	dum--;

      dum++;
    }

    kvd->insert_val(act,dum-act);
    act= dum;
  }
  return true;
}

bool ValueReader::append_from_file(char const* fname, char const* block) {
  ifstream is(fname);
  if (!is) { 
    cerr << "\nCannot open control file" << fname;
    return false;
  }
  const int maxLineLength= 2023;

  char line[maxLineLength+1];
  char* act;
  int lineNo=0;
  //cerr << "\nParsing file \"" << fname << "\" with block [" << block << "] ";
  bool in_active_block= false;
  int block_size= 0;
  if (block) block_size= strlen(block);

  if (block_size == 0) //
    in_active_block= true;

  while (is) {
    is.getline(line,maxLineLength); lineNo++;
    
    act=line;
    while (isspace(*act)) act++; //strip leading whitespace

    // Check if comment
    if (*act==comment_char) continue; // OK, comment
    if (*act=='[') { // OK, recognizes [block]
      in_active_block= false;
      act++;
      //cout << "\n act= " << act << "\n blockdata()= " << block.data() << "\n block_size= " << block_size;
      if ( 0==strncmp(act, block, block_size) ) {
	act+= block_size;
	if ( ']'== *act )
	  in_active_block= true;
      }
      continue; 
    }

    if (!in_active_block)
      continue;

    append_from_string(act);
  }
  return true;
}

int ValueReader::get(char const* key, char & val, char const* & next) {
  int res= get(key,next);
  if ( res <= 0 )
    return res;
  
  val= *next;
  next++;
  strspace(next,next);
  return 1;
}

int ValueReader::get(char const* key, char * val, int num, char const* & next) {
  int res= get(key,next);
  if ( res <= 0 || num < 1)
    return res;

  if ( res < num) {
    strncpy(val,next,res);
    val[res]= '\0';
    next+= res;
    return res;
  }
  
  strncpy(val,next,num-2);
  val[num-1]= '\0';
  next += num-1;
  return num-1;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/******************************* T E S T *************************************/

#if 0

void test() {
#if 0
  const int maxLineLength= 1024;
  const char fname[]= "test.txt";
  ifstream is(fname);
  if (!is) { 
    cerr << "\nCannot open control file" << fname;
    exit(1);
    //return;
  }

  char line[maxLineLength+1];

  char fun[20];
  int size;
  int rand;
  double err;

  while (is) {
    is.getline(line,maxLineLength); 
    const char * dum= line;
    while ( isspace(*dum) )
      dum++;

    if ( *dum == '\0' || *dum == comment_char ) //comment or blank line
      continue; 

    ValueParser vp(0,line);
    
    vp.get("fun",fun,20);
    vp.get("size",size, -1);
    vp.get("rand",rand,-1);
    int res= vp.get("err",err);
    if (res != 1 || size < 0 || rand < 0) {
      cerr << "\nerrornous line [" << line << "]";
    }
    else
      cout << "\n" << fun << " | " << size << " | " << rand << " -> " << err;
  }
  cout << endl;
#endif
}


int main (int argc,char **argv) {
  if (argc<= 2) {
    cout << "\nneed one argument";
    return 1;
  }
  ValueReader P;
  P.verbose= false;
  P.append_from_file(argv[1]);
  argc--;
  argv++;
		     
  P.append_from_command_line(argc-1,argv+1);
  //P.set_verbose(true);
  cout << P << flush;
  //return 1;
  char chr;
  bool h;
  P.get("help",h);
  P.get("char",chr);

  //int i;
  bool b_arr[4];
  int i_arr[4];
  long l_arr[4];
  //char c_arr[4];
  //char c2_arr[4];
  float f_arr[4];
  double d_arr[4];

  int res;
  char const* next;

  res= P.get("l",l_arr,4,next);   
  cout << "\n res= " << res << " next= [" << next << "]\n----------";
  return 1;
  res= P.get("b",b_arr,4);  
  cout << "\n res= " << res << "\n----------";
#if 0
  res= P.get("c",c_arr,4,'\0');    
  cout << "\n res= " << res << "\n----------";
  res= P.get("c2",c2_arr,4,"123");
  cout << "\n res= " << res << "\n----------";
#endif
  res= P.get("i",i_arr,4);   
  cout << "\n res= " << res << "\n----------";
  res= P.get("f",f_arr,4);
  cout << "\n res= " << res << "\n----------";
  res= P.get("d",d_arr,4);
  cout << "\n res= " << res << "\n----------";

  cout << endl;

  //P.show_not_accessed_entries(cerr);
  //cerr << "\nnumber of accessed entries= " << P.num_of_accessed_entries() << endl;
  return 0;
}

#endif
