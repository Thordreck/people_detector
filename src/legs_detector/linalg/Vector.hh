#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <macros.h>
#include <string.h>


#ifndef VECTOR_HH
#define VECTOR_HH

/*! 
 * \class Vector
 * \brief A vector class.
 */
class Vector
{
public:

  //! Default Constructor
  Vector ();
  explicit Vector (uint const len);
  explicit Vector (double const *data, uint const len);
  Vector (Vector const &other);

  //! Destructor.
  virtual ~Vector();

  //! Returns class name.
  virtual bool IsA(char const *name) const
  { return (strcmp(name, "Vector") == 0); }

  //! ???
  void PushBack(double d); // not yet implemented

  //! Vector Size.
  inline uint Size() const { return _size; };

  Vector& operator=(Vector const & other);
  virtual const double &operator[] (int i) const throw (Exception *);
  virtual double &operator[] (int i) throw (Exception *);

  bool operator== (Vector const & other) const;
  bool operator!= (Vector const &other) const;

  Vector operator+ (Vector const & other) const throw (Exception *);
  Vector operator- (Vector const & other) const throw (Exception *);
  Vector operator* (double x) const;
  Vector operator/ (double x) const throw (Exception *);
  Vector operator- () const;
  operator double*() const { return _data; };
  friend Vector operator* (double x, Vector const &v);
  friend Vector operator/ (double x, Vector const &v) throw (Exception *);

  Vector& operator+= (Vector const & other) throw (Exception*);
  Vector& operator-= (Vector const & other) throw (Exception*);
  Vector& operator*= (double x);
  Vector& operator/= (double x);

  //! Returns the maximum.
  double Max() const;
  //! Returns the minimum.
  double Min() const;
  //! Returns the argument of the maximum.
  uint ArgMax() const;
  //! Returns the argument of the minimum.
  uint ArgMin() const;
  //! Returns the length.
  double Norm() const;
 //! The sum
  double Sum() const;
  //! ???
  double Enorm() const;
  //! ???
  double NormSqr() const;
  //! Returns mean of values in Vector.
  double Mean() const;
  //! Returns variance of values in Vector.
  double Var() const;
  //! Returns standard deviation of values in Vector.
  double VarSqrt() const;

  //! Normalizes the vector and returns it.
  Vector &Unit() throw (Exception *);
  //! Returns the angle between to vectors.
  double Angle(Vector const &other) const;
  //! Returns the dot product from two vectors.
  double Dot(Vector const &other) const throw (Exception *);

  //! Gram-Schmidt orthonormalization.
  static void Orthonorm(Vector *vec_array, uint size) throw (Exception *);

  //! Vector filled with ones.
  static Vector Ones(uint m);
  //! Vector filled with zeros.
  static Vector Zeros(uint m);
  //! Random vector.
  static Vector Random(uint m);

protected:

  virtual void copy(Vector const &other);
  void copy(const double *data,uint len);
  virtual void destroy();

  //private:

  uint _size;
  double *_data;

};

ostream &operator<<(ostream &stream, Vector const &vec);

#endif
