/******************************************************************
 *
 * $Revision: 1.2 $
 * $Date: 2005-10-03 14:13:36 +0200 (Mo, 03 Okt 2005) $
 * $Author: triebel $
 * $Log: Exception.hh,v $
 * Revision 1.2  2004/06/19 08:08:07  rschmidt
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2004/06/18 10:21:33  triebel
 * Initial import
 *
 * Revision 1.2  2004/04/14 16:08:17  triebel
 * Patched Exception.hh
 *
 * Revision 1.1  2003/10/07 16:37:42  rtriebel
 * Imported new repository (second part).
 *
 * Revision 1.3  2003/03/27 22:49:03  rtriebel
 * Changed the Makefiles. Made the entire code
 * "g++-3.2  compliant".
 *
 * Revision 1.2  2003/03/22 03:22:51  rtriebel
 * Class "exception" changed.
 *
 * Revision 1.1.1.1  2002/11/21 17:20:07  rtriebel
 * Imported sources
 *
 *
 ******************************************************************/

using namespace std;

#include <string>
#include <iostream>
#include <sstream>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef EXCEPTION_HH
#define EXCEPTION_HH


typedef enum {
  WARNING, ERROR, FATAL, UNDEF
} ErrorSeverity;

class Exception;

typedef void (*ErrorHandler)(const Exception *);

class Exception
{
public:
  void Handle() const;

  // Set custom error handler (if not set, the default handler is used)
  static void SetErrorHandler(ErrorHandler const &new_handler);

  // Resets to the default handler
  static void ResetErrorHandler(ErrorHandler const &new_handler);

  // Returns error message
  const string  GetMessage()    const { return message;};

  virtual ~Exception() {}

  ErrorSeverity severity;

protected:

  Exception(char const *msg, ErrorSeverity sev ) : 
    severity(sev), message(msg), filename(), line_number(0) {cerr << msg << endl;}

  Exception(char const *msg, char const *fname, uint ln_nb, 
	    ErrorSeverity sev ) : 
    severity(sev), message(msg), filename(fname), line_number(ln_nb) { cerr << msg << endl;}
  
  Exception(char const *msg);

  const string &GetFileName()   const { return filename; }
  uint          GetLineNumber() const { return line_number; }
  ErrorSeverity GetSeverity()   const { return severity; }
  bool          FileName()      const { return (!filename.empty()); }
  bool          LineNumber()    const { return (line_number != 0);}

  virtual void DefaultHandler() const = 0;

  void SetMessage(va_list list);

  va_list arg_list;

private:
  
  string message;
  string filename;
  uint line_number;
};

class Warning : public Exception
{
public:
  Warning(char const *msg, ...) : Exception(msg, WARNING) 
  { 
    va_start(arg_list, msg);
    SetMessage(arg_list);
  }

  Warning(char const *msg, char const *fname, uint ln_nb, ...) : 
    Exception(msg, fname, ln_nb, WARNING) 
  { 
    va_start(arg_list, ln_nb);
    SetMessage(arg_list);
  }

private:

  virtual void DefaultHandler() const;
};

class Error : public Exception
{
public:
  Error(char const *msg, ...) : Exception(msg, ERROR)
  { 
    va_start(arg_list, msg);
    SetMessage(arg_list);
  }

  Error(char const *msg, char const *fname, uint ln_nb, ...) : 
    Exception(msg, fname, ln_nb, ERROR)
  { 
    va_start(arg_list, ln_nb);
    SetMessage(arg_list);
  }

private:

  virtual void DefaultHandler() const;
};

class Fatal : public Exception
{
public:

  Fatal(char const *msg, ...) : Exception(msg, FATAL)
  { 
    va_start(arg_list, msg);
    SetMessage(arg_list);
  }

  Fatal(char const *msg, char const *fname, uint ln_nb, ...) : 
    Exception(msg, fname, ln_nb, FATAL)
  { 
    va_start(arg_list, ln_nb);
    SetMessage(arg_list);
  }

private:

  virtual void DefaultHandler() const;
};

#endif
