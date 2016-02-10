#include <Vector.hh>

#define GIANT 1.304e19
#define DWARF 3.834e-20

Vector::Vector(): _size(0), _data(NULL) {}

Vector::Vector (uint const len)
{
  _size = len;
  _data = new double[len];

  for (uint i=0 ; i<len ; ++i ) {
     _data[i] = 0;
  }
}

Vector::Vector (double const *data, uint const len)
{
  _size = 0;
  _data = NULL;

  copy( data, len );
}

Vector::Vector (Vector const &other)
{
  _size = 0;
  _data = NULL;

  copy( other );
}

Vector::~Vector()
{
  destroy();
}

Vector& Vector::operator= (Vector const &other)
{
  if (this != &other) {
    copy( other );
  }

  return *this;
}

const double &Vector::operator[] (int i) const throw (Exception *)
{
  if (i < 0) {
    throw new Error("Index into vector is negative.", __FILE__, __LINE__);
  }
  if ( i >= (int)this->Size() ) {
    throw new Error("Index exceeds Vector dimension.", __FILE__,__LINE__);
  }

  return _data[i];
}

double &Vector::operator[] (int i) throw (Exception *)
{
  if (i < 0) {
    throw new Error("Index into vector is negative.", __FILE__, __LINE__);
  }
  if (i >= (int)this->Size()) {
    throw new Error("Index exceeds Vector dimension.", __FILE__,__LINE__);
  }

  return _data[i];
}

bool Vector::operator== (Vector const &other) const
{
  return ((*this - other).NormSqr() < PREC * PREC );
}

bool Vector::operator!= (Vector const &other) const
{
  return !(*this == other);
}

Vector Vector::operator+ (Vector const &other) const throw (Exception *)
{
  if ( Size() != other.Size() ) {
    throw new Error("Vector dimensions do not agree.", __FILE__,__LINE__);
  }

  Vector sum(other);
  for ( uint i=0; i < sum.Size(); ++i ) {
    sum[i] += _data[i];
  }

  return sum;
}

Vector Vector::operator- (Vector const &other) const throw (Exception *)
{
  if ( Size() != other.Size()) {
    throw new Error("Vector dimensions do not agree.", __FILE__,__LINE__);
  }

  Vector diff(*this);
  double *otherdata = other;

  for (uint i=0; i<diff.Size(); i++) {
    diff[i]-= otherdata[i];
  }

  return diff;
}

Vector Vector::operator* (double x) const
{
  Vector prod(*this);

  for(uint i=0; i<prod.Size(); i++) {
    prod[i] *= x;
  }

  return prod;
}

Vector operator* (double x, Vector const &v)
{
  return v * x;
}

Vector operator/ (double x, Vector const &v) throw (Exception *)
{
  Vector out(v);

  for(uint i=0; i<out.Size();i++) {
    if ( fabs(out[i]) < PREC) {
      throw new Error("Division by zero",__FILE__,__LINE__);
    } else {
      out._data[i] = x / out._data[i];
    }
  }

  return out;
}

Vector Vector::operator/ (double x) const throw (Exception *)
{
  if ( fabs(x) < PREC ) {
    throw new Error("Division by zero.",__FILE__,__LINE__);
  }

  Vector quot(*this);
  for(uint i=0; i<quot.Size(); i++) {
    quot._data[i] /= x;
  }

  return quot;
}

Vector Vector::operator-() const
{
  Vector minus(*this);

  for(uint i=0; i<minus.Size(); i++) {
    minus._data[i] = -minus._data[i];
  }

  return minus;
}

Vector& Vector::operator+= (Vector const &other) throw (Exception*)
{
  NO_CHECK(*this = *this + other);
  return *this;
}

Vector& Vector::operator-= (Vector const &other) throw (Exception*)
{
  NO_CHECK(*this = *this - other);
  return *this;
}

Vector& Vector::operator*= (double x)
{
  *this = *this * x;
  return *this;
}

Vector& Vector::operator/= (double x)
{
  *this = *this / x;
  return *this;
}

double Vector::Max() const
{
  double m = _data[0];

  for( uint i = 1; i < Size(); ++i ) {
    if ( _data[i] > m ) {
       m = _data[i];
    }
  }

  return m;
}

double Vector::Min() const
{
  double m = _data[0];

  for( uint i = 1; i < Size(); ++i ) {
    if ( _data[i] < m ) {
       m = _data[i];
    }
  }

  return m;
}

uint Vector::ArgMax() const
{
  uint m = 0;

  for(uint i=1; i<this->Size(); i++)
    if((*this)[i] > (*this)[m])
      m = i;

  return m;
}

uint Vector::ArgMin() const
{
  uint m = 0;

  for(uint i=1; i<this->Size(); i++)
    if((*this)[i] < (*this)[m])
      m = i;

  return m;
}

double Vector::Sum() const
{
  double sum = 0;
  for( uint i = 0; i < Size(); ++i ) {
    sum += _data[i];
  }
  return sum;
}

double Vector::Norm() const
{
  return sqrt(NormSqr());
}

double Vector::Mean() const
{
  double mean = 0;
  
  for ( uint i=0; i<this->Size(); ++i ) {
    mean += _data[i];
  }
  return mean/(double)( Size() );
}

double Vector::Var() const
{
  double mean = Mean();
  double var = 0;

  for ( uint i = 0; i < Size(); ++i ) {
    var += SQR(_data[i] - mean);
  }
  return var/(double)( Size() );
}

double Vector::VarSqrt() const
{
  return sqrt( Var() );
}

double Vector::Enorm() const
{
  double s1 = 0., s2 = 0., s3 = 0.;
  double x1max = 0., x3max = 0., xabs = 0.;
  double agiant = GIANT / (double) this->Size();

  for (uint i = 0; i < Size(); ++i ) {
    xabs = fabs( _data[i] );

    if (xabs >= agiant) {

      /*  sum for large components. */
      if (xabs > x1max) {
        s1 = 1. + s1 * SQR(x1max / xabs);
        x1max = xabs;
      } else {
        s1 += SQR(xabs / x1max);
      }
    } else if (xabs <= DWARF){

      /* sum for small components. */
      if (xabs > x3max) {
        s3 = 1 + s3 * SQR(x3max / xabs);
        x3max = xabs;
      } else if (xabs != 0.) {
        s3 += SQR(xabs / x3max);
      }
    } else {
      /* sum for intermediate components. */
      s2 += SQR(xabs);
    }
  }
  /* calculation of norm. */

  if (s1 != 0.) {
    return x1max * sqrt(s1 + s2 / x1max / x1max);
  }

  if (s2 == 0. ) {
    return x3max * sqrt(s3);
  }

  if (s2 >= x3max) {
    return sqrt(s2 + SQR(x3max) * s3);
  }

  return sqrt(x3max * (s2 / x3max + x3max * s3));
}

double Vector::NormSqr() const
{
  double nsq = 0;

  for (uint i=0; i<this->Size(); i++)
    nsq += SQR((*this)[i]);

  return nsq;
}

Vector &Vector::Unit() throw (Exception *)
{
  double len = Norm();

  //if ( len < PREC)
  //  throw new Error("Division by zero.", __FILE__, __LINE__);

  if(len >= PREC)
    *this /= len;

  return *this;
}

double Vector::Angle(Vector const &other) const
{
  double c = this->Dot(other)/(this->Norm() * other.Norm());

  if (c >= 1.0)
    return 0.;
  else if (c <= -1.0)
    return M_PI;
  
  return acos(c);
}

double Vector::Dot(Vector const &other) const throw (Exception *)
{
  if (this->Size() != other.Size())
    throw new Error("Vector dimensions do not agree.", __FILE__, __LINE__);

  double x = 0;
  for (uint i=0; i<this->Size(); i++)
    x += (*this)[i] * other[i];

  return x;
}


void Vector::Orthonorm(Vector *vec_array, uint size) throw (Exception *)
{
    // If the input vectors are v0, v1, and v2, then the Gram-Schmidt
    // orthonormalization produces vectors u0, u1, and u2 as follows,
    //
    //   u0 = v0/|v0|
    //   u1 = (v1-(u0*v1)u0)/|v1-(u0*v1)u0|
    //   u2 = (v2-(u0*v2)u0-(u1*v2)u1)/|v2-(u0*v2)u0-(u1*v2)u1|
    //
    // where |A| indicates length of vector A and A*B indicates dot
    // product of vectors A and B.

  if (size == 0 || vec_array == 0) return;

  uint len0 = vec_array[(uint)0].Size();
  for (uint i = 0; i<size; i++)
    if (vec_array[i].Size() != len0)
      throw new Error("Vector dimensions do not agree.",__FILE__,__LINE__);

  for (uint i = 0; i<size; i++){
    for (uint j = 0; j<i; j++)
      vec_array[i] -= vec_array[j].Dot(vec_array[i]) * vec_array[j];
    vec_array[i].Unit();
  }
}

Vector Vector::Ones(uint m)
{
  Vector out(m);

  for(uint i=0; i<m; i++)
    out[i] = 1;
  return out;
}

Vector Vector::Zeros(uint m)
{
  Vector out(m);

  for(uint i=0; i<m; i++)
    out[i] = 0;
  return out;
}

Vector Vector::Random(uint m)
{
  double samples[m];
  struct timeval tv;
  struct timezone tz;

  gettimeofday(&tv, &tz);
  srand(tv.tv_usec);
  for(uint i=0; i<m; i++)
    samples[i] = rand() /(RAND_MAX + 1.);

  return Vector(samples, m);
}

void Vector::copy(Vector const &other)
{
  copy(other._data,other.Size());
}

void Vector::copy(const double *data,uint len)
{
  if (data == NULL)
    return;

  if (_size != len)
  {
    destroy();
    _size = len;
    _data = new double[_size];
  }

  memcpy(_data, data, _size * sizeof(double));
}
void Vector::destroy()
{
  if (_data != NULL)
    delete[] _data;

  _data = NULL;
  _size = 0;
}

ostream &operator<<(ostream &stream, Vector const &vec)
{
  //stream.precision(13);
  //stream.setf(ios::fixed, ios::floatfield);

  for(uint i=0; i<vec.Size(); i++)
    stream << setw(7) << vec[i] << " ";
  stream << flush;

  return stream;
}
