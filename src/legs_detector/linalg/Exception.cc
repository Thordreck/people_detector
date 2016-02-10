/******************************************************************
 *
 * $Revision: 1.1 $
 * $Date: 2004-06-18 12:21:32 +0200 (Fr, 18 Jun 2004) $
 * $Author: triebel $
 * $Log: Exception.cc,v $
 * Revision 1.1  2004/06/18 10:21:33  triebel
 * Initial revision
 *
 * Revision 1.1  2003/10/07 16:37:42  rtriebel
 * Imported new repository (second part).
 *
 * Revision 1.2  2003/03/22 03:22:51  rtriebel
 * Class "exception" changed.
 *
 * Revision 1.1.1.1  2002/11/21 17:20:07  rtriebel
 * Imported sources
 *
 *
 ******************************************************************/

#include <Exception.hh>
#include <memory>
#include <stdarg.h>

ErrorHandler custom_handler;
bool custom_handler_defined = false;

void Exception::SetMessage(va_list list)
{  
  uint n = vsnprintf(0, 0, message.c_str(), list);
  std::auto_ptr<char> buf(new char[n + 1]);
  vsprintf(buf.get(), message.c_str(), list);

  message = string(buf.get());
}


void Exception::Handle() const
{
  if (!custom_handler_defined)
    DefaultHandler();
  else
    custom_handler(this);
}

void Exception::SetErrorHandler(ErrorHandler const &new_handler)
{
  custom_handler = new_handler;
  custom_handler_defined = true;
}

void Exception::ResetErrorHandler(ErrorHandler const &new_handler)
{
  custom_handler = 0;
  custom_handler_defined = false;
}

void Warning::DefaultHandler() const
{
  cerr << "Warning!" << endl;
  if (FileName())
    cerr << "In file \"" << GetFileName() << "\" " << flush;
  if (LineNumber())
    cerr << "(line number " << GetLineNumber() << ")" << flush;
  if (FileName() || LineNumber())
    cerr << ": " << flush;
  cerr << GetMessage() << endl;
}

void Error::DefaultHandler() const
{
  cerr << "Error!" << endl;
  if (FileName())
    cerr << "In file \"" << GetFileName() << "\" " << flush;
  if (LineNumber())
    cerr << "(line number " << GetLineNumber() << ")" << flush;
  if (FileName() || LineNumber())
    cerr << ": " << flush;
  cerr << GetMessage() << endl;
}

void Fatal::DefaultHandler() const
{
  cerr << "Fatal error!" << endl;
  if (FileName())
    cerr << "In file \"" << GetFileName() << "\" " << flush;
  if (LineNumber())
    cerr << "(line number " << GetLineNumber() << ")" << flush;
  if (FileName() || LineNumber())
    cerr << ": " << flush;
  cerr << GetMessage() << endl;
  cerr << "Exiting" << endl;
  delete this;
  exit(1);
}
