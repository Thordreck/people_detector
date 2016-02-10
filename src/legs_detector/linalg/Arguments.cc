/******************************************************************
 *                                                                
 * $Revision: 1.1 $                                               
 * $Date: 2004-06-18 12:21:32 +0200 (Fr, 18 Jun 2004) $                                   
 * $Author: triebel $                                            
 * $Log: Arguments.cc,v $
 * Revision 1.1  2004/06/18 10:21:33  triebel
 * Initial revision
 *
 * Revision 1.1  2003/10/07 16:37:42  rtriebel
 * Imported new repository (second part).
 *
 * Revision 1.3  2003/03/22 09:55:07  rtriebel
 * Upgrade to g++ version 3.2.
 *
 * Revision 1.2  2002/12/18 16:38:04  rtriebel
 * Added texture server.
 *
 * Revision 1.1.1.1  2002/11/21 17:20:07  rtriebel
 * Imported sources
 *
 *
 ******************************************************************/

#include <Arguments.hh>

/////////////////////////////////////////
//
// Class Arg
//

//////// public /////////

Arg::Arg(const string usage)
{
  int l = usage.length();
  int start = 0;
  int end = l-1;

  // First check if argument is optional
  optional = 0;
  if (end > 0){
    if (usage[start] == '[')
      if (usage[end] == ']'){
	start++;
	end--;
	optional = 1;
      }
      else
	err();

    // Skip whitespace:
    while ( (start < end) && (usage[start++] == ' ') );
    start--; 
    while ( (end  > start) && (usage[end--] == ' ') );
    end++;
    if (start >= end) err();
    if (usage[start] == '-'){
      identifier = usage[++start];
      while ( (start < end) && (usage[++start] ==' '));
    }
    else{
      // Optional arguments without identifiers must be of type string
      identifier = '\0';
      if ( (optional) && (usage[start] != '$'))
	err();
    }
    switch (usage[start]){
    case('$'):{
      if (usage[end] == '$'){
	type = "string";
	if (++start >= --end) err();
	name = usage.substr(start, end-start+1);
      }
      else
	err();
      break;
    }
    case('#'):{
      if (usage[end] == '#'){
	type = "int";
	if (++start >= --end) err();
	name = usage.substr(start,end-start+1);
      }
      else
	err();
      break;
    }
    case('*'):{
      if (usage[end] == '*'){
	type = "float";
	if (++start >= --end) err();
	name = usage.substr(start,end-start+1);
      }
      else
	err();
      break;
    }
    default:
      type = "bool";
      name = "";
    }
  }
  else
    err();
}


ostream &operator<<(ostream &stream, Arg const &a)
{
  stream << "Argument identifier: " << a.GetId() << endl;
  stream << "Argument name: " << a.GetName() << endl;
  stream << "Argument type: " << a.GetType() << endl;
  stream << "Argument is optional: " << (a.IsOpt()? "yes" : "no") << endl;

  return stream;
	 
}

template <class DataType>
ostream &operator<<(ostream &stream, Opt<DataType> const &o)
{
  stream << "Option name: " << o.GetName() << endl;
  stream << "Option value: " << o.GetValue() << endl;

  return stream;
	 
}


/////////////////////////////////////////
//
// Class Arguments
//

//////// public /////////

Arguments::Arguments(const string usage) : 
  idx_wtht_id(-1), prog_name(), arglist(), 
  str_opts(), fl_opts(), int_opts(), bool_opts()
{
  int start = 0, end = usage.length()-1;
  int until;
  
  // Skip whitespace
  start = usage.find_first_not_of(' ',start);
 
  while ((start < end) && (start >=0)) {
    switch (usage[start]){
    case ('['):
       until = usage.find_first_of(']',start+1);
       break;
    case ('$'):
      until = usage.find_first_of('$',start+1);
      break;
    case ('#'):
      until = usage.find_first_of('#',start+1);
      break;
    case ('*'):
      until = usage.find_first_of('*',start+1);
      break;
    case ('-'):
      until = usage.find_first_of("[-$#*",start+1);
      if (until < 0) 
	until = start+1;
      else {
	switch(usage[until]){
	case ('['):
	  until--;
	  break;
	case ('-'):
	  until--;
	  break;	  
	case('$'):
	  until = usage.find_first_of('$',until+1);
	  break;
	case('*'):
	  until = usage.find_first_of('*',until+1);
	  break;
	case('#'):
	  until = usage.find_first_of('#',until+1);
	  break;
	}
      }
      break;
    default:
      err();
    }
    Arg arg( usage.substr(start, until-start+1) );
    arglist.push_back( arg );
    start = until + 1;
    // Skip whitespace
    start = usage.find_first_not_of(' ',start);
  }
  if (arglist.size() == 0){
    //errflg = 2;
    Arg dummy;
    arglist.push_back(dummy);
  }
}       

bool Arguments::Parse(int argc, char **argv)
{
  if (*argv == 0) return 0;
  prog_name = argv[0];

  int idx;
  bool parsed[arglist.size()];

  // Initialization
  for (uint i=0; i<arglist.size(); i++) parsed[i] = false;

  for (int arg_idx = 1; arg_idx < argc; arg_idx++){
    string opt(argv[arg_idx]);
    if(opt[0] == '-'){
      idx = get_arg_idx(opt[1]);
      if (idx == -1) {
	cerr << Usage() << endl;
	return 0;
      } 
      if (!(parsed[idx])){
	// Argument with identifier follows     
	if (opt.length() == 2){
	  // The option consists only of the identifier
	  // This means that the next string in 'argv' contains
	  // the parameter of the option, except if the argument
	  // is of type 'bool'
	  if (arglist[idx].GetType() == "string"){
	    if (++arg_idx == argc) err();
	    Opt<string> str(arglist[idx].GetName(),argv[arg_idx]);
	    str_opts.push_back(str);
	  }
	  else if (arglist[idx].GetType() == "float"){
	    if (++arg_idx == argc) err();
	    float tmp;
	    sscanf(argv[arg_idx],"%f",&tmp);
	    Opt<float> fl(arglist[idx].GetName(),tmp);
	    fl_opts.push_back(fl); 
	  }
	  else if (arglist[idx].GetType() == "bool"){
	    string tmp;
	    tmp = arglist[idx].GetId();
	    Opt<bool> bl(tmp,1);
	    bool_opts.push_back(bl) ;}
	  else if (arglist[idx].GetType() == "int"){
	    if (++arg_idx == argc) err();
	    int j;
	    sscanf(argv[arg_idx],"%i",&j);
	    Opt<int> in(arglist[idx].GetName(),j);
	    int_opts.push_back(in);
	  }
	}
	else{
	  // The actual option string has more than 2 characters.
	  // This means that every single character represents a
	  // boolean option (flag), or that the parameter of this
	  // option is given by 'opt[2] ... opt[opt.length()-1]' 
	  if (arglist[idx].GetType() == "bool"){
	    string tmp;
	    tmp = arglist[idx].GetId();
	    Opt<bool> bl(tmp,1);
	    bool_opts.push_back(bl);
	    unsigned int str_idx = 2;
	    int idx2;
	    while (((idx2 = get_arg_idx(opt[str_idx++])) != -1) &&
		   (arglist[idx2].GetType() == "bool") && 
		   (str_idx <= opt.length())){
	      tmp = arglist[idx2].GetId();
	      Opt<bool> bl2(tmp,1);
	      bool_opts.push_back(bl2);
	    }
	    if (str_idx <= opt.length()) {
	      cerr << Usage() << endl;
	      return 0;
	    }
	  } 
	  else if (arglist[idx].GetType() == "string"){
	    Opt<string> str(arglist[idx].GetName(),opt.substr(2));
	    str_opts.push_back(str);
	  }
	  else if (arglist[idx].GetType() == "float"){
	    float tmp;
	    sscanf(opt.substr(2).c_str(),"%f",&tmp);
	    Opt<float> fl(arglist[idx].GetName(),tmp);
	    fl_opts.push_back(fl);
	  }
	  else if (arglist[idx].GetType() == "int"){
	    int tmp;
	    sscanf(opt.substr(2).c_str(),"%i",&tmp);
	    Opt<int> in(arglist[idx].GetName(),tmp);
	    int_opts.push_back(in);
	  }
	}
	parsed[idx] = 1;
      }
      else if (arglist[idx].GetType() != "bool")
	// If argument already parsed and not of type bool, then 
	// increment argument counter
	arg_idx++;
    }
    else{
      // The option string does not start with a '-'.
      // This means that the actual option string represents
      // an argument without identifier. In order to detect, 
      // which of the possible arguments without identifier
      // is meant here, we search the list of arguments in 
      // the order of their appearance. 

	// Move pointer to argument without identifier one step forward
	next_arg_wtht_id();
	// Check whether index is in valid range

	if (( idx_wtht_id < (int)arglist.size() ) && !( parsed[idx_wtht_id])){
	  if (arglist[idx_wtht_id].GetType() == "string"){
	    Opt<string> str(arglist[idx_wtht_id].GetName(),opt);
	    str_opts.push_back(str);
	  } 
	  else if (arglist[idx_wtht_id].GetType() == "float"){
	    float tmp;
	    sscanf(opt.c_str(),"%f",&tmp);
	    Opt<float> fl(arglist[idx_wtht_id].GetName(),tmp);
	    fl_opts.push_back(fl);	    
	  }
	  else if (arglist[idx_wtht_id].GetType() == "int"){
	    int tmp;
	    sscanf(opt.c_str(),"%i",&tmp);
	    Opt<int> in(arglist[idx_wtht_id].GetName(),tmp);
	    int_opts.push_back(in);	    
	  }	    
	parsed[idx_wtht_id] = 1;
	}
	else{
	  cerr << Usage() << endl;
	  return 0;
	}
    }
  }
  for (unsigned int i=0; i < arglist.size(); i++){
    if ( !(arglist[i].IsOpt()) && !(parsed[i])){
      cerr <<  Usage() << endl;
      return 0;
    }
  }
  return 1;
}

string Arguments::Usage() const
{
  unsigned int line_break = 45;
  string usage_str = "Usage: " + prog_name + " ";
  for (unsigned int i=0; i<arglist.size(); i++){
    if (arglist[i].IsOpt())
      usage_str += '[';
    if (arglist[i].GetId() != '\0'){
      usage_str += '-';
      usage_str += arglist[i].GetId();
      usage_str += ' ';
    }
    usage_str+=arglist[i].GetName();
    if (arglist[i].IsOpt())
      usage_str += ']';
    usage_str +=' ';
    if (usage_str.length() > line_break){
      usage_str  += "\n\t";
      line_break *= 2;
    }
  }
  return usage_str;
}

//////// private /////////

int Arguments::get_arg_idx(char id) const
{
  for (unsigned int i = 0; i < arglist.size(); i++)
    if (arglist[i].GetId() == id)
      return i;
  return -1;
}

int Arguments::GetIntOpt(char const *name, int def) const
{
  for(unsigned int k = 0; k < int_opts.size(); k++)
    if (int_opts[k].GetName() == name)
      return (int_opts[k].GetValue());
  return def;
}

float Arguments::GetFloatOpt(char const *name, float def) const
{
  for(unsigned int k = 0; k < fl_opts.size(); k++)
    if (fl_opts[k].GetName() == name)
      return (fl_opts[k].GetValue());
  return def;
}

bool Arguments::GetBoolOpt(char const *name, bool def) const
{
  for(unsigned int i = 0; i<bool_opts.size(); i++)
    if (bool_opts[i].GetName() == name)
      return (bool_opts[i].GetValue());
  return def;
}

string Arguments::GetStrOpt(char const *name, string def) const
{
  for(unsigned int i = 0; i<str_opts.size(); i++)
    if (str_opts[i].GetName() == name)
      return (str_opts[i].GetValue());
  return def;
}


ostream &operator<<(ostream &stream, Arguments const &args)
{
  for (unsigned int k=0; k< args.get_args().size(); k++)
    stream << args.get_args()[k] << endl;
  return stream;
}
