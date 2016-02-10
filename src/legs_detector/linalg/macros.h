/******************************************************************
 *
 * $Revision: 1.1 $
 * $Date: 2004-12-22 18:40:41 +0100 (Mi, 22 Dez 2004) $
 * $Author: triebel $
 * $Log: macros.h,v $
 * Revision 1.1  2004/06/18 10:21:33  triebel
 * Initial revision
 *
 * Revision 1.3  2003/11/17 14:36:48  rtriebel
 * Added a new macro.
 *
 * Revision 1.2  2003/11/14 17:17:16  jomeyer
 * added macros for printing
 *
 * Revision 1.1  2003/10/07 16:58:55  rtriebel
 * Makefile etc
 *
 * Revision 1.6  2003/03/11 23:02:19  rtriebel
 * *** empty log message ***
 *
 * Revision 1.5  2003/02/04 00:16:02  rtriebel
 * Added 3d exploration for IPC. Many other updates.
 *
 * Revision 1.4  2003/01/20 16:11:14  rtriebel
 *
 * minor changes
 *
 * Revision 1.3  2002/12/18 16:38:04  rtriebel
 * Added texture server.
 *
 * Revision 1.2  2002/12/09 18:00:35  rtriebel
 * Added Canon support into AMTEC control. Adjusted pantilt server.
 *
 * Revision 1.1.1.1  2002/11/21 17:20:07  rtriebel
 * Imported sources
 *
 *
 ******************************************************************/

#ifndef MACROS_H
#define MACROS_H

#include <math.h>

#ifdef __cplusplus

#include <Exception.hh>
#include <Arguments.hh>

#define READ_FILE(fvar,fname) \
   ifstream fvar(fname);\
   if (!fvar) throw new Error("Could not open file %s",fname);
#define WRITE_FILE(fvar,fname) \
   ofstream fvar(fname);\
   if (!fvar) throw new Error("Could not open file %s",fname);
#define WRITE_VRML_FILE(fvar,fname) \
   ofstream fvar(fname);\
   if (!fvar) throw new Error("Could not open file %s",fname);\
   fvar << "#VRML V2.0 utf8\n";
#define FSKIP_LINE(f) \
   {char c=' ';while(c != '\n' && !f.eof()) f.get(c);}
#define FGET_LINE(f,l) \
   {l = ""; char c=' '; while(c!='\n' && !f.eof()) l+=c=f.get();}
#define FREAD(f,a) \
   {f>>a; if (f.fail()) throw new Error("Parse error.",__FILE__,__LINE__);}
#define CHECK(expr) \
   try{expr;} catch(Exception *e){e->Handle(); delete e;}
#define CHECK_EXIT(expr) \
   try{expr;} catch(Exception *e){e->Handle(); delete e; exit(1);}
#define CHECK_RETURN(expr) \
   try{expr;} catch(Exception *e){e->Handle(); delete e; return;}
#define NO_CHECK(expr) \
   try{expr;} catch(...){throw;}
#define ARGMAX_VEC(v,a) \
   for(uint _i=a=0; _i<(v).size(); _i++) a=((v)[_i]>(v)[a]?_i:a);
#define ARGMIN_VEC(v,a) \
   for(uint _i=a=0; _i<(v).size(); _i++) a=((v)[_i]<(v)[a]?_i:a);
#define DEL_FEXT(fname) \
   {uint d = fname.find_last_of('.'); \
    if (d!=string::npos) fname = fname.substr(0,d);}
#define GET_FEXT(fname,fext) \
   {uint d = fname.find_last_of('.'); \
    fext = (d!=string::npos) ? fname.substr(d+1) : "";}
#define BEGIN_PROGRAM(ac,av) \
   int main(int ac, char **av){ \
     try
#define END_PROGRAM \
     catch(Exception *e){ \
       e->Handle(); delete e; exit(1);\
     }\
   }

template <class KeyType, class ValueType>
  class CompareLess{
 public:
  bool operator()(pair<KeyType, ValueType> const &p1, 
		  pair<KeyType, ValueType> const &p2){
    return (p1.second < p2.second);
  }
 };

template <class KeyType, class ValueType>
  class CompareGreater{
 public:
  bool operator()(pair<KeyType, ValueType> const &p1, 
		  pair<KeyType, ValueType> const &p2){
    return (p1.second > p2.second);
  }
 };
#endif

#ifndef GET_COLOR
#define GET_COLOR(i, size, r, g, b) \
  ((floor(i*6./(double)size) == 0) || (floor(i*6./(double)size) == 6) ? \
   (r=1,g=(i*6./size-floor(i*6./size)),b=0) : \
   ((floor(i*6./(double)size) == 1) ? (r=1.-(i*6./size-floor(i*6./size)),g=1,b=0) : \
    ((floor(i*6./(double)size) == 2) ? (r=0,g=1,b=(i*6./size-floor(i*6./size))) : \
     ((floor(i*6./(double)size) == 3) ? (r=0,g=1-(i*6./size-floor(i*6./size)), b=1) : \
      ((floor(i*6./(double)size) == 4) ? (r=(i*6./size-floor(i*6./size)),g=0, b=1) : \
       (r=1,g=0, b=1-(i*6./size-floor(i*6./size))))))))
#endif

#define PREC 1e-12
#define SQRT3 1.732050807568877293527446341505872366943

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MIN3
#define MIN3(a,b,c) MIN((a),MIN((b),(c)))
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef MAX3
#define MAX3(a,b,c) MAX((a),MAX((b),(c)))
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)/180.*M_PI)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)/M_PI*180.)
#endif

#ifndef SQR
#define SQR(x) ((x)*(x))
#endif

#ifndef CUB
#define CUB(x) ((x)*(x)*(x))
#endif

#ifndef HYPOT
#define HYPOT(x,y) sqrt(SQR(x) + SQR(y))
#endif

#ifndef ROUND
#define ROUND(x) ((x) < 0 ? (int)((x) - .5): (int) ((x) + .5))
#endif

#ifndef IS_INT
#define IS_INT(a) (fabs(ROUND(a)-(a))<PREC)
#endif

#ifndef SIGN
#define SIGN(a) ((a)>0?1:((a)<0?-1:0))
#endif

#ifndef PNT_DIST
#define PNT_DIST(a,b) (fabs(a-b)/a)
#endif

#ifndef NORM_ANGLE
#define NORM_ANGLE(theta) (((theta) >= M_PI && (theta) < M_PI) ? (theta) : (((theta) -= ((int)((theta)/(2*M_PI))) * 2 * M_PI) >= M_PI ? ((theta) -= 2*M_PI) : ((theta) < -M_PI ? ((theta) += 2*M_PI) : (theta) )))
#endif

#define AMTEC_DEFAULT_VEL .5
#define AMTEC_DEFAULT_ACC .4

#define COUT \
cout <<
#define TAB \
<< "\t" <<
#define SPACE \
<< " " <<
#define ENDL \
<< endl;

#endif

