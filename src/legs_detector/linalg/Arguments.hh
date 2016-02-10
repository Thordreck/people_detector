/******************************************************************
 *                                                                
 * $Revision: 1.1 $                                               
 * $Date: 2004-06-18 12:21:32 +0200 (Fr, 18 Jun 2004) $                                   
 * $Author: triebel $                                            
 * $Log: Arguments.hh,v $
 * Revision 1.1  2004/06/18 10:21:33  triebel
 * Initial revision
 *
 * Revision 1.2  2003/11/02 12:59:48  rtriebel
 * Added an example file into arguments
 *
 * Revision 1.1  2003/10/07 16:37:42  rtriebel
 * Imported new repository (second part).
 *
 * Revision 1.3  2003/03/22 09:55:07  rtriebel
 * Upgrade to g++ version 3.2.
 *
 * Revision 1.2  2003/01/20 16:11:14  rtriebel
 *
 * minor changes
 *
 * Revision 1.1.1.1  2002/11/21 17:20:07  rtriebel
 * Imported sources
 *
 *
 ******************************************************************/

#ifndef ARGUMENTS_HH
#define ARGUMENTS_HH

#include <macros.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <vector>

template<class DataType>
class Opt
{
public:
  Opt() : name(), value() {};
  Opt(string s) : name(s), value() {};
  Opt(string s, DataType v): name(s), value(v) {};
  const string &GetName() const { return name; };
  const DataType &GetValue() const { return value; };

private:
  string   name;
  DataType value;
};

class Arg
{
public:

  Arg(): name(), identifier(0), type(), optional(0) {};
  Arg(const string usage);

  const char   &GetId()   const { return identifier; };
  const string &GetType() const { return type; };
  const string &GetName() const { return name; };
  const bool   &IsOpt()   const { return optional; };

private:
  string name;
  char   identifier;
  string type;
  bool   optional;
  void err() { 
    cerr << "Parse error" << endl;
    exit(1);
  }
};

ostream &operator<<(ostream &, Arg const &);

template <class DataType>
ostream &operator<<(ostream &, Opt<DataType> const &);


class Arguments 
{
public:

  // Description:
  // The constructor takes a usage string from the command line.
  // This string has a the following format: 
  //    1. Command line options can have an identifier, which
  //       is a single upper or lower caser letter, or a name,
  //       which should describe the option clearly, or both a
  //       name or an identifier. A name must have more than one 
  //       character.
  //    2. The data type of a command line option is defined by
  //       surrounding the option name with a special character.
  //       This is: 
  //       a.  For floating point: *
  //       b.  For integer values: #
  //       c.  For string options: $
  //       Boolean options don't have a name, they consist only
  //       of the identifier.
  //    3. Optional arguments are surrounded by brackets. Boolean 
  //       options should always be surrounded by brackets.
  // 
  // Example:
  // Arguments args("[-o $outfile$] $infile$ [-d] *delta*");
  // args.Parse();
  //
  // This program will expect a mandatory option "infile" of type
  // string, another mandatory option "delta" of type float, an optional
  // argument "outfile" of type string, and a boolean option with identifier
  // "d". Every other call to the program will result in an error,
  // where the usage is generated automatically and sent to sdterr.

  Arguments(const string usage);
  bool Parse(int, char **);

  // Description:
  // Returns program name
  const string &GetProgName() const { return prog_name; };

  // Description:
  // Generates usage string
  string Usage() const;

  // Description:
  // After parsing the command line, the expected options can
  // be retrieved with these functions. Every function expects
  // a name of the corresponding option and a default value in case
  // the argument was not given in the command line.
  int    GetIntOpt  (char const *name, int def = 0) const;
  bool   GetBoolOpt (char const *name, bool def = 0) const;
  float  GetFloatOpt(char const *name, float def = 0) const;
  string GetStrOpt (char const *name, string def = "") const;

  // Description:
  // Insertion operator to show all arguments
  friend ostream &operator<<(ostream &, Arguments const &);

private:

  // Private helper functions
  vector<Arg> const &get_args() const {return arglist;}
  int get_arg_idx(char) const;

  // Inline functions
  void next_arg_wtht_id() {
    int size = arglist.size();
    while( (++idx_wtht_id < size ) && 
	   (arglist[idx_wtht_id].GetId() != '\0') ); 
  };
  void err() { cerr << "Parse error" << endl << Usage() << endl; exit(1); };

  int idx_wtht_id;
  string prog_name;
  vector<Arg> arglist;
  vector<Opt<string> > str_opts;
  vector<Opt<float> >  fl_opts;
  vector<Opt<int> >    int_opts;
  vector<Opt<bool> >   bool_opts;

};

ostream &operator<<(ostream &, Arguments const &);

#endif
