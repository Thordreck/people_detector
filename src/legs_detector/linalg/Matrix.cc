#include <Matrix.hh>

Matrix::Matrix() : 
  _m(0), _n(0), _size(0), _data(0), _type(GENERIC), eigen_vec_computed(false){}

Matrix::Matrix(uint m, uint n, MatrixType type) : 
  _m(m), _n(n), _size(m*n), _data(0), _type(type), eigen_vec_computed(false)
{
  _data = (double*)calloc(_size,sizeof(double));
}

Matrix::Matrix(uint m, MatrixType type) : 
  _m(m), _n(m), _size(m*m), _data(0), _type(type), eigen_vec_computed(false)
{
  _data = (double*)calloc(_size,sizeof(double));
}

Matrix::Matrix(double *data, uint m, uint n, MatrixType type) : 
  _m(m), _n(n), _size(m*n), _data(0), _type(type), eigen_vec_computed(false)
{
  _data = (double*)calloc(_size,sizeof(double));
  memcpy (_data, data, _size*sizeof(double));
}

Matrix::Matrix(double *data, uint m, MatrixType type) : 
  _m(m), _n(m), _size(m*m), _data(0), _type(type), eigen_vec_computed(false)
{
  switch(_type){
  case(DIAGONAL):
    _size = _m;
    break;
  case(SYMMETRIC):
  case(U_TRIANGULAR):
    _size = _m*(_m+1)/2;
    break;
  case(GENERIC):
    _size = _m*_n;
  }

  _data = (double*)calloc(_size,sizeof(double));
  memcpy (_data, data, _size*sizeof(double));
}

Matrix::Matrix(Matrix const &other) : 
  _m(0), _n(0), _size(0), _data(0), _type(GENERIC), eigen_vec_computed(false)
{
  copy(other);
}

Matrix::~Matrix()
{
  destroy();
}

Matrix Matrix::Identity(uint m, uint n)
{
  Matrix I(m,n);
  
  for(uint i=0; i<MIN(m,n); i++)
    I[i][i] = 1.;
  
  return I;
}

Matrix Matrix::Diag(double *data, uint len)
{
  Matrix m(len, len);
  for(uint i=0; i<len; i++)
    m[i][i] = data[i];
  return m;
  //return Matrix(data, len, DIAGONAL);
}

Matrix Matrix::Random(uint m, uint n)
{
  Matrix out(m,n);
  struct timeval tv;
  struct timezone tz;

  gettimeofday(&tv, &tz);
  srand(tv.tv_usec);
  for(uint i=0; i<out.Rows(); i++)
    for(uint j=0; j<out.Cols(); j++)
      out[i][j] = rand() /(RAND_MAX + 1.);
  return out;
}

Matrix Matrix::Ones(uint m, uint n)
{
  Matrix out(m,n);

  for(uint i=0; i<m; i++)
    for(uint j=0; j<n; j++)
      out[i][j] = 1;
  return out;
}

Matrix Matrix::Zeros(uint m, uint n)
{
  Matrix out(m,n);

  for(uint i=0; i<m; i++)
    for(uint j=0; j<n; j++)
      out[i][j] = 0;
  return out;
}

Matrix Matrix::OutProd(Vector const &v)
{
  Matrix out(v.Size(), v.Size());

  for(uint i=0; i<v.Size(); i++)
    for(uint j=0; j<v.Size(); j++)
      out[i][j] = v[i] * v[j];

  return out;
}

Matrix
Matrix::OutProd(Vector const &v, Vector const &w)
{
  Matrix out(v.Size(), w.Size());

  for(uint i=0; i<v.Size(); i++)
    for(uint j=0; j<w.Size(); j++)
      out[i][j] = v[i] * w[j];

  return out;
}

Matrix &Matrix::operator=(Matrix const &other)
{
  if (this != &other){
    destroy();
    copy(other);
  }
  return *this;
}

double *Matrix::operator[](uint m) throw (Exception *)
{
  if (m >= _m) throw new Error("Index exceeds matrix dimensions.",
			       __FILE__, __LINE__);
  return &_data[m*_n];
}

const double *Matrix::operator[](uint m) const throw (Exception *)
{
  if (m >= _m) throw new Error("Index exceeds matrix dimensions,",
			       __FILE__, __LINE__);
  return &_data[m*_n];
}

Vector Matrix::Row(uint i) const throw (Exception *)
{
  if (i>=_m) throw new Error("Index exceeds matrix dimensions.",
			     __FILE__,__LINE__);

  return Vector(&_data[i*_n],_n);
}

Vector Matrix::Col(uint i) const throw (Exception *)
{
  if (i>=_n) throw new Error("Index exceeds matrix dimensions.",
			     __FILE__,__LINE__);

  Vector c(_m);
  for(uint row = 0; row < _m; row++)
    c[row] = (*this)[row][i];

  return c;
}

double Matrix::Max() const
{
  double m = _data[0];
  
  for(uint i=1; i<_size; i++)
    m = _data[i] > m ? _data[i] : m;

  return m;
}

double Matrix::Min() const
{
  double m = _data[(uint)0];
  
  for(uint i=1; i<_size; i++)
    m = _data[i] < m ? _data[i] : m;

  return m;

}

void Matrix::Write(const char *filename) const throw (Exception *)
{
  WRITE_FILE(file,filename);

  file << this->Rows() << " " << this->Cols() << endl;
  
  for(uint i=0; i<_m; i++){
    for(uint j=0; j<_n; j++)
      file << (*this)[i][j] << " ";
    file << endl;
  }
  file.close();
}

void Matrix::Read(const char *filename) throw (Exception *)
{
  READ_FILE(file,filename);

  file >> _m;
  file >> _n;
  
  _size = _n * _m;
  
  //cerr << "Matrix::Read(...) : " << endl;
  //cerr << "    _m: " << _m << endl;
  //cerr << "    _n: " << _n << endl;
  
  if ( !file ) {
  	char msg[500];
	sprintf(msg, "ERROR: Matrix::Read(...): reading matrix from file %s", filename);
  	throw new Error("Error reading matrix from file \n");
  }
  
  uint index;
  _data = new double[_n * _m];
  for( uint i=0; i < _m; i++) {
	for( uint j=0; j < _n; j++) {  
		index = i * _n + j;
		file >> _data[index]; 
		
		//cerr << "index: " << index << endl;
		//cerr << i << "," << j << ": " << _data[index] << endl;
		
		if ( !file ) {
	  		char msg[500];
			sprintf(msg, "ERROR: Matrix::Read(...): reading matrix from file %s", filename);
  			throw new Error("Error reading matrix from file \n");
		}
	}  		
  }
  
  
  file.close();
}

void Matrix::ExportPGM(const char *filename) const throw (Exception *)
{
  double max = this->Max(), min = this->Min();
  WRITE_FILE(file,filename);

  file << "P5" << endl;
  file << "# Matrix (by rtriebel)" << endl;
  file << _n << " " << _m << endl;
  file << "255" << endl;

  char buf[_n];
  for(uint i=0; i<_m; i++){
    for(uint j=0; j<_n; j++)
      buf[j] = (char) (((*this)[i][j] - min) / (max- min) * 255.);
    file.write(buf, _n * sizeof(char));
  }
  file.close();
}

Matrix Matrix::operator+ (double x) const
{
  Matrix sum(_m, _n);
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      sum[row][col] = (*this)[row][col] + x;

  return sum;
}

Matrix Matrix::operator+ (Matrix const &other) const throw (Exception *)
{
  if (_m != other.Rows() || _n != other.Cols())
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  Matrix sum(_m, _n);
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      sum[row][col] = (*this)[row][col] + other[row][col];

  return sum;
}

Matrix &Matrix::operator+=(double x)
{
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      (*this)[row][col] += x;

  return *this;
}

Matrix &Matrix::operator+=(Matrix const &other) throw (Exception *)
{
  if (_m != other.Rows() || _n != other.Cols())
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      (*this)[row][col] += other[row][col];

  return *this;
}

Matrix Matrix::operator- (double x) const
{
  Matrix dif(_m, _n);
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      dif[row][col] = (*this)[row][col] - x;

  return dif;
}

Matrix Matrix::operator- (Matrix const &other) const throw (Exception *)
{
  if (_m != other.Rows() || _n != other.Cols())
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  Matrix diff(_m, _n);
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      diff[row][col] = (*this)[row][col] - other[row][col];

  return diff;
}

Matrix &Matrix::operator-=(double x)
{
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      (*this)[row][col] -= x;

  return *this;
}

Matrix &Matrix::operator-=(Matrix const &other) throw (Exception *)
{
  if (_m != other.Rows() || _n != other.Cols())
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      (*this)[row][col] -= other[row][col];

  return *this;
}

Matrix Matrix::Translate(Vector const &vec, char const *dir) throw (Exception *)
{
  Matrix out(_m, _n);

  if (!strcmp(dir,"ROW")) {
    if (_n != vec.Size())
      throw new Error("Matrix and Vector dimensions do not agree.",__FILE__,__LINE__);

    for(uint row = 0; row < _m; row++)
	  for(uint col = 0; col < _n; col++)
	    out[row][col] = (*this)[row][col] + vec[col];
  }
  else if (!strcmp(dir,"COL")) {
    if (_m != vec.Size())
      throw new Error("Matrix and Vector dimensions do not agree.",__FILE__,__LINE__);

    for(uint row = 0; row < _m; row++)
	  for(uint col = 0; col < _n; col++)
	    out[row][col] = (*this)[row][col] + vec[row];
  }

  return out;
}

Matrix Matrix::operator* (Matrix const &other) const throw (Exception *)
{
  if (_n != other.Rows() )
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  Matrix prod(_m, other.Cols());
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < other.Cols(); col++){
      prod[row][col] = 0.;
      for (uint i=0; i < _n; i++)
      prod[row][col] += (*this)[row][i]*other[i][col];
    }

  return prod;
}

Vector Matrix::operator* (Vector const &vec) const throw (Exception *)
{
  if (vec.Size() != _n)
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Vector prod(_m);
  for (uint row = 0; row < _m; row++){
    prod[row] = 0;
    for (uint col = 0; col < _n; col++)
      prod[row] += (*this)[row][col] * vec[col];
  }

  return prod;
}
Matrix Matrix::operator* (double x) const
{
  Matrix prod(_m, _n);
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      prod[row][col] = x*(*this)[row][col];
  
  return prod;
}

Matrix &Matrix::operator*=(double x)
{
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      (*this)[row][col] *= x;
  
  return *this;
}

Vector operator* (Vector const &vec, Matrix const &mat) throw (Exception *)
{
  if (vec.Size() != mat.Cols())
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Vector prod(mat.Cols());
  for (uint col = 0; col < mat.Cols(); col++){
    prod[col] = 0.;
    for (uint row = 0; row < mat.Rows(); row++)
      prod[col] += vec[row] * mat[row][col];
  }
  
  return prod;
}

Matrix operator* (double x, Matrix const &mat)
{
  return mat * x;
}

Matrix Matrix::operator/ (double x) const throw (Exception *)
{
  if (fabs(x) < PREC)
    throw new Error("Division by zero",__FILE__, __LINE__);

  Matrix out(*this);

  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      out[row][col] = (*this)[row][col] / x;
  return out;
}

Matrix &Matrix::operator/=(double x) throw (Exception *)
{
  if (fabs(x) < PREC)
    throw new Error("Division by zero",__FILE__, __LINE__);

  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      (*this)[row][col] /= x;

  return *this;
}

Matrix operator/ (double x, Matrix const &mat) throw (Exception *)
{
  Matrix out(mat);

  for (uint row = 0; row < out.Rows(); row++)
    for (uint col = 0; col < out.Cols(); col++)
      if (fabs(mat[row][col]) < PREC)
	throw new Error("Division by zero",__FILE__, __LINE__);
      else
	out[row][col] = x / mat[row][col];

  return out;
}

Matrix Matrix::operator- () const
{
  Matrix neg(_m, _n);
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < _n; col++)
      neg[row][col] = -(*this)[row][col];

  return neg;
}


Matrix Matrix::operator|(Vector const &vec) const throw (Exception *)
{
  if (_m != vec.Size()) 
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix out(_m,_n+1);
  for(uint i=0; i<_m; i++){
    memcpy(out[i],(*this)[i],_n * sizeof(double));
    out[i][_n] = vec[i];
  }

  return out;
}

Matrix Matrix::operator|=(Vector const &vec) throw (Exception *)
{
  NO_CHECK((*this) = (*this) | vec);
  return (*this);
}

Matrix operator|(Vector const &vec1, Vector const &vec2) throw (Exception *)
{
  if (vec1.Size() != vec2.Size())
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix out(vec1.Size(),2);
  for(uint i=0; i<vec1.Size(); i++){
    out[i][0] = vec1[i];
    out[i][1] = vec2[i];
  }
  return out;
}

Matrix operator|(Vector const &vec, Matrix const &mat) throw (Exception *)
{
  if (vec.Size() != mat.Rows())
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix out(vec.Size(), mat.Cols()+1);
  for(uint i=0; i<vec.Size(); i++){
    out[i][0] = vec[i];
    memcpy(&out[i][1],mat[i],mat.Cols() * sizeof(double));
  }
  return out;
}


Matrix Matrix::operator|(Matrix const &other) const throw (Exception *)
{
  if (_m != other.Rows()) 
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix out(_m,_n+other.Cols());
  for(uint i=0; i<_m; i++){
    memcpy(out[i],(*this)[i],_n * sizeof(double));
    memcpy(&out[i][_n],other[i],other.Cols() * sizeof(double));
  }

  return out;
}

Matrix Matrix::operator|=(Matrix const &other) throw (Exception *)
{
  NO_CHECK(*this = (*this) | other);
  return *this;
}

Matrix Matrix::operator,(Vector const &vec) const throw (Exception *)
{
  if (_n != vec.Size())
    throw new Error("Matrix/Vector dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix out(_m+1, _n);
  memcpy(out[0], (*this)[0], this->Size() * sizeof(double));
  memcpy(out[this->Rows()], vec, vec.Size() * sizeof(double));

  return out;
}

Matrix Matrix::operator,(Matrix const &other) const throw (Exception *)
{
  if (_n != other.Cols())
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  Matrix out(_m+other.Rows(), _n);
  memcpy(out[0], (*this)[0], this->Size() * sizeof(double));
  memcpy(out[this->Rows()], other[0], other.Size() * sizeof(double));

  return out;
}

Matrix Matrix::Submat(uint i1, uint j1, uint i2, uint j2) const
  throw (Exception *)
{
  if(i1 > i2 || j1 > j2) 
    throw new Error("Invalid submatrix indices.",__FILE__,__LINE__);
  if(i1 >= _m || i2 >= _m || j1 >= _n || j2 >= _n)
    throw new Error("Submatrix index exceeds matrix dimensions.",
		    __FILE__,__LINE__);

  Matrix sub(i2-i1+1, j2-j1+1);
  
  for(uint i=0; i<sub.Rows(); i++)
    memcpy(sub[i], &(*this)[i1 + i][j1], (j2-j1+1)*sizeof(double));

  return sub;
}

Matrix Matrix::Flip(char const *dir)
{
  Matrix out(_m, _n);

  if (!strcmp(dir,"ROWS"))
    for(uint i=0; i<_m; i++)
      memcpy(out[i], (*this)[_m-i-1], _n*sizeof(double));
  else if (!strcmp(dir,"COLS"))
    for(uint i=0; i<_m; i++)
      for(uint j=0; j<_n; j++)
	out[i][j] = (*this)[i][_n-j-1];

  return out;
}

Matrix Matrix::Transp() const
{
  Matrix trans(_n, _m);
  for (uint row = 0; row < _n; row++)
    for (uint col = 0; col < _m; col++)
      trans[row][col] = (*this)[col][row];

  return trans;
}

Matrix Matrix::TranspTimes(Matrix const & other) const throw (Exception *)
{
  if (_m != other.Rows()) 
    throw new Error("Matrix dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix prod(_n, other.Cols());
  for (uint row = 0; row < _n; row++)
    for (uint col = 0; col < other.Cols(); col++){
      prod[row][col] = 0.0f;
      for (uint i = 0; i < _m; i++)
	prod[row][col] += (*this)[i][row]*other[i][col];
    }
  return prod;
}

Matrix Matrix::TimesTransp(Matrix const & other) const throw (Exception *)
{
  if (_n != other.Cols()) 
    throw new Error("Matrix dimensions do not agree.",
		    __FILE__,__LINE__);

  Matrix prod(_m, other.Rows());
  for (uint row = 0; row < _m; row++)
    for (uint col = 0; col < other.Rows(); col++){
      prod[row][col] = 0.0f;
      for (uint i = 0; i < _n; i++)
	prod[row][col] += (*this)[row][i] * other[col][i];
    }

  return prod;
}

double Matrix::ElemMult(Matrix const &other) const throw (Exception *)
{
  if (_m != other.Rows() || _n != other.Cols())
    throw new Error("Matrix dimensions do not agree.",__FILE__,__LINE__);

  double sum = 0;
  for(uint i=0; i<_m; i++)
    for(uint j=0; j<_n; j++)
      sum += (*this)[i][j] * other[i][j];

  return sum;
}

double Matrix::Fnorm() const
{
  double sumsq = 0.0;

  for (uint i = 0; i < _m; i++ )
    for (uint j = 0; j < _n; j++ )
      sumsq += SQR((*this)[i][j]);

   return sqrt(sumsq);
}

Matrix Matrix::Invert() const throw (Exception *)
{
   if (_m != _n ) 
     throw new Error("Matrix must be square.",__FILE__,__LINE__);

   int s;
   gsl_matrix *mat = gsl_matrix_alloc(_m, _n);
   gsl_matrix *inv = gsl_matrix_alloc(_m, _n);
   gsl_permutation * p = gsl_permutation_alloc (_m);

   memcpy(mat->data, _data, _size * sizeof(double));

   gsl_linalg_LU_decomp(mat, p, &s);
   gsl_linalg_LU_invert(mat, p, inv);

   Matrix Inv(inv->data, _m, _n);

   gsl_matrix_free(mat);
   gsl_matrix_free(inv);
   gsl_permutation_free(p);

   return Inv;
}

Matrix Matrix::Pinv() const throw (Exception *)
{
  Matrix U,D,VT;
  
  NO_CHECK(SVD(U,D,VT));

  for (uint i = 0; i<MIN(_m,_n); i++)
    D[i][i] = (fabs(D[i][i]) < PREC ? 0 : 1./D[i][i] );

  return VT.Transp() * D.Transp() * U.Transp();
}

void Matrix::QRFac(Matrix &Q, Matrix &R) const throw (Exception *)
{
  gsl_vector *tau = gsl_vector_alloc(MIN(_m,_n));
  gsl_matrix *a = gsl_matrix_alloc(_m, _n);
  gsl_matrix *q = gsl_matrix_alloc(_m, _m);
  gsl_matrix *r = gsl_matrix_alloc(_m, _n);

  memcpy(a->data, _data, _size * sizeof(double));

  gsl_linalg_QR_decomp(a, tau);
  gsl_linalg_QR_unpack(a, tau, q, r);

  Q = Matrix(q->data, _m);
  R = Matrix(r->data, _m, _n);

  gsl_vector_free(tau);
  gsl_matrix_free(a);
  gsl_matrix_free(q);
  gsl_matrix_free(r);  
}

void Matrix::SVD(Matrix &U, Matrix &D, Matrix &VT) const throw (Exception *)
{
  if (_m == 0 || _n == 0) return;
  if (_m < _n)
    throw new Error("SVD of MxN matrix with m < n not implemented.",
		    __FILE__,__LINE__);

  gsl_matrix *u  = gsl_matrix_alloc(_m, _n);
  gsl_vector *dd = gsl_vector_alloc(_n);
  gsl_vector *wk = gsl_vector_alloc(_n);
  gsl_matrix *v  = gsl_matrix_alloc(_n, _n);

  memcpy(u->data, _data, _size * sizeof(double));

  gsl_linalg_SV_decomp(u, v, dd, wk);
  gsl_matrix_transpose(v);

  U = Matrix(u->data, _m, _n);
  D = Matrix(_n);
  for(uint i=0; i<_n; i++)
    D[i][i] = dd->data[i];
  VT = Matrix(v->data, _n, _n);

  gsl_matrix_free(u);
  gsl_vector_free(dd);
  gsl_vector_free(wk);
  gsl_matrix_free(v);
}

void Matrix::Chol(Matrix &L) const throw (Exception *)
{
  if (_m != _n)
    throw new Error("Cholesky decomposition of a non-square matrix.",
		    __FILE__,__LINE__);
  
  int err;
  gsl_matrix *l = gsl_matrix_alloc(_m, _n);
  memcpy(l->data, _data, _size * sizeof(double));

  err = gsl_linalg_cholesky_decomp(l);
  L = Matrix(l->data, _m);
  for(uint i=0; i<_m; i++)
    for(uint j=i+1; j<_n; j++)
      L[i][j] = 0;

  gsl_matrix_free(l);
  
  if (err == GSL_EDOM)
    throw new Error("Cholesky decomposition of a non-positive definite "
		    "matrix.",__FILE__,__LINE__);
}

double Matrix::Trace() const
{
  double tr = 1;

  for(uint i=0; i<MIN(_m,_n); i++)
    tr *= (*this)[i][i];
  
  return tr;
}

double Matrix::Det() const throw (Exception *)
{
  if (_m != _n) return 0;

  int s;
  double dd;
  gsl_matrix *a = gsl_matrix_alloc(_m, _n);
  gsl_permutation *p = gsl_permutation_alloc(_m);
  
  memcpy(a->data, _data, _size * sizeof(double));
  
  gsl_linalg_LU_decomp(a, p, &s);
  dd = gsl_linalg_LU_det(a, s);

  gsl_matrix_free(a);
  gsl_permutation_free(p);

  return dd;
}

double Matrix::Sum() const
{
  double sum = 0;
  for(uint i=0; i<_size; i++)
    sum += _data[i];
  return sum;
}

Matrix Matrix::Convolve(Matrix const &kernel) const throw (Exception *)
{
  if (kernel.Rows() > _m || kernel.Cols() > _n)
    throw new Error("Invalid kernel dimensions.",__FILE__,__LINE__);

  Matrix kernel_flip(kernel);
  Matrix out(*this);

  uint m = kernel.Rows(), n = kernel.Cols();
  uint mid_m = (m-1)/2, mid_n = (n-1)/2; 
  for(uint i=0; i<m; i++)
    for(uint j=0; j<n; j++)
      kernel_flip[i][j] = kernel[m-i-1][n-j-1];
  
  int idx1, idx2;
  double val, sum;
  for(uint i=0; i<_m; i++)
    for(uint j=0; j<_n; j++){
      sum = 0.;
      for(int k = -(int)mid_m; k<= (int)mid_m; k++)
	for(int l = -(int)mid_n; l<= (int)mid_n; l++){
	  idx1 = k + i;  idx2 = l + j;
	  if(idx1 < 0 || idx2 < 0 || idx1 >= (int)_m || idx2 >= (int)_n)
	    val = 0.;
	  else
	    val = (*this)[(uint)idx1][idx2] * 
	      kernel[(uint)(k+mid_m)][(uint)(l+mid_n)];
	  sum += val;
	}
      out[i][j] = sum;
    }
  return out;
}

double Matrix::Interpolate(double x, double y) const throw (Exception *)
{
  if (x < 0. || y < 0. || x > _m - 1.|| y > _n - 1.)
    throw new Error("Index exceeds matrix dimensions.",__FILE__,__LINE__);
  
  uint ix, iy, iix, iiy;
  ix = (uint)x; iix = ix + 1;
  iy = (uint)y; iiy = iy + 1;
  if (iix >= _m) iix = ix;
  if (iiy >= _n) iiy = iy;

  double r, c, ic, ir;
  r = x - ix; ir = 1 - r;
  c = y - iy; ic = 1 - c;

  return (ir * ic * (*this)[ix][iy]  + r * ic * (*this)[iix][iy] +
	  ir *  c * (*this)[ix][iiy] + r *  c * (*this)[iix][iiy]);
}

Vector Matrix::Mean(int dim) const
{
  Vector mean_vec;
  if (dim == 0){
    mean_vec = this->Row(0);
    for(uint i=1; i<this->Rows(); i++)
      mean_vec += this->Row(i);
    mean_vec /= (double)this->Rows();
  }
  else{
    mean_vec = this->Col(0);
    for(uint i=0; i<this->Cols(); i++)
      mean_vec += this->Col(i);
    mean_vec /= (double)this->Cols();
  }

  return mean_vec;
}

Matrix Matrix::AutoCov(int dim) const
{
  Vector mean = Mean(dim);
  Matrix C;
  
  if(dim == 0){
    
    C = Matrix(this->Cols(), this->Cols());

    for(uint i=0; i<this->Rows(); i++)
      C += OutProd(this->Row(i));
    C /= (double)this->Rows();
    C -= OutProd(mean);
  }
  else {

    C = Matrix(this->Rows(), this->Rows());

    for(uint i=0; i<this->Cols(); i++)
      C += OutProd(this->Col(i));
    C /= (double)this->Cols();
    C -= OutProd(mean);
  }
  return C;
}

Matrix Matrix::CrossCov(Matrix const &other, int dim) const
{
  if (this->Rows() != other.Rows() || 
      this->Cols() != other.Cols())
    throw new Error("Matrix dimensions do not match", 
		    __FILE__, __LINE__);

  Vector mean1 = Mean(dim);
  Vector mean2 = other.Mean(dim);

  Matrix C;
  
  if(dim == 0){

    C = Matrix(this->Cols(), this->Cols());

    for(uint i=0; i<this->Rows(); i++)
      C += OutProd(this->Row(i), other.Row(i));
    C /= (double)this->Rows();
    C -= OutProd(mean1, mean2);
  }

  else {

    C = Matrix(this->Rows(), this->Rows());

    for(uint i=0; i<this->Cols(); i++)
      C += OutProd(this->Col(i), other.Col(i));
    C /= (double)this->Cols();
    C -= OutProd(mean1, mean2);
  }

  return C;
}


void Matrix::EigenSolve(Vector &eval, Matrix &evec, bool real) throw (Exception*)
{
  if (_m != _n)
    throw new Error("Matrix must be square.",__FILE__,__LINE__);

  bool is_symmetric = true;

  for(uint i=0; i<_m; i++)
    for(uint j=i; j<_m; j++)
      if (fabs((*this)[i][j]-(*this)[j][i]) > PREC)
	is_symmetric = false;

  if (is_symmetric){
    
    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc (_m);
    gsl_matrix *a   = gsl_matrix_alloc(_m, _m);
    gsl_vector *val = gsl_vector_alloc (_m);
    gsl_matrix *vec = gsl_matrix_alloc (_m, _m);
    
    memcpy(a->data, _data, _size * sizeof(double));
    gsl_eigen_symmv (a, val, vec, w);
    gsl_eigen_symmv_sort (val, vec, GSL_EIGEN_SORT_ABS_DESC);
  
    if (real)
      eval = Vector(val->data, _m);
    else
      eval = Vector(_m);
    evec = Matrix(vec->data, _m);
    
    gsl_eigen_symmv_free(w);
    gsl_matrix_free(a);
    gsl_vector_free(val);
    gsl_matrix_free(vec);
  }
  else{

    // Allocate work space

    if (!eigen_vec_computed){

      H = new double*[_n];
      V = new double*[_n];
      ort = new double [_n];
      d   = new double [_n];
      e   = new double [_n];
      
      
      for(uint i=0; i<_n; i++){

	H[i] = new double[_n];
	V[i] = new double[_n];

	for(uint j=0; j<_n; j++){
	  H[i][j] = (*this)[i][j];
	  V[i][j] = 0.;
	}
	ort[i] = d[i] = e[i] = 0.;
      }
      
      orthes();

      hqr2();

      eigen_vec_computed = true;
    }
    
    if (real == true)
      eval = Vector(d, _n);
    else
      eval = Vector(e, _n);
    
    evec = Matrix(_n, _n);

    for(uint i=0; i<_n; i++)
      for(uint j=0; j<_n; j++)
	evec[i][j] = V[i][j];
  }
  
}

void Matrix::EigenSolveSymm(uint m, Vector const &data,
			    Vector &eval, Matrix &evec, bool sort)
{
  double eval_data[m];
  double evec_data[m * m];

  EigenSolveSymm(m, (double*)data, eval_data, evec_data, sort);
  eval = Vector(eval_data, m);
  evec = Matrix(evec_data, m, m);
}

void Matrix::EigenSolveSymm(uint m, double *data,
			    double *eval, double *evec, bool sort)
{
  gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc (m);
  gsl_matrix *a   = gsl_matrix_alloc(m, m);
  gsl_vector *val = gsl_vector_alloc (m);
  gsl_matrix *vec = gsl_matrix_alloc (m, m);

  double *entries = new double[m*m];
  uint k=0;
  for(uint i=0; i<m; i++)
    for(uint j=0; j<m; j++)
      if (j>= i)
        entries[i*m+j] = data[k++];
      else
        entries[i*m+j] = entries[j*m+i];

  memcpy(a->data, entries, m * m * sizeof(double));
  gsl_eigen_symmv (a, val, vec, w);

  if (sort)
    gsl_eigen_symmv_sort (val, vec, GSL_EIGEN_SORT_ABS_DESC);

  memcpy(eval, val->data, m * sizeof(double));
  memcpy(evec, vec->data, m * m * sizeof(double));

  delete entries;

  gsl_eigen_symmv_free(w);
  gsl_matrix_free(a);
  gsl_vector_free(val);
  gsl_matrix_free(vec);
}


void Matrix::destroy()
{
  if (_data) free(_data);

  if (eigen_vec_computed){
    for(uint i=0; i<_n; i++){
      delete [] H[i];
      delete [] V[i];
    }
    delete [] H;
    delete [] V;
    delete [] ort;
  }
}


void Matrix::copy(Matrix const &other)
{
  _m = other.Rows();
  _n = other.Cols();
  if (other.Type() == DIAGONAL)
    _size = _m;
  else if (other.Type() == SYMMETRIC || other.Type() == U_TRIANGULAR)
    _size = _m*(_m+1)/2;
  else
    _size = _m * _n;

  if (_size != 0){
    _data = (double*)calloc(_size,sizeof(double));
    memcpy(_data, other[0], _size * sizeof(double));
  }
  else
    _data = NULL;
}

ostream &operator<<(ostream &stream, Matrix const &mat)
{
  stream.precision(3);
  stream.setf(ios::fixed, ios::floatfield);
  stream << mat.Rows() << "x" <<mat.Cols() << " Matrix:" << endl;
  for(uint i=0; i<mat.Rows(); i++){
    stream << "\t";
    for(uint j=0; j<mat.Cols(); j++)
      stream << setw(7) << mat[i][j] << " ";
    stream << endl;
  }

  return stream;
}

void Matrix::cdiv(double xr, double xi, double yr, double yi, double &cdivr, double &cdivi) 
{
  double r,dd;

  if (fabs(yr) > fabs(yi)) {

    r = yi/yr;
    dd = yr + r*yi;
    cdivr = (xr + r*xi)/dd;
    cdivi = (xi - r*xr)/dd;
  } 
  else {
    r = yr/yi;
    dd = yi + r*yr;
    cdivr = (r*xr + xi)/dd;
    cdivi = (r*xi - xr)/dd;
  }
}

// Nonsymmetric reduction to Hessenberg form.
void Matrix::orthes () 
{
  if (_m != _n)
    throw new Error("Matrix must be square!", __FILE__, __LINE__);

  
  //  This is derived from the Algol procedures orthes and ortran,
  //  by Martin and Wilkinson, Handbook for Auto. Comp.,
  //  Vol.ii-Linear Algebra, and the corresponding
  //  Fortran subroutines in EISPACK.
  
  uint low  = 0;
  uint high = _n-1;
  
  for (uint m = low+1; m <= high-1; m++) {
    
    // Scale column.
    
    double scale = 0.0;
    for (uint i = m; i <= high; i++) 
      scale += fabs(H[i][m-1]);

    if (scale != 0.0) {
   
      // Compute Householder transformation.
      
      double h = 0.0;
      for (uint i = high; i >= m; i--) {
	ort[i] = H[i][m-1]/scale;
	h += ort[i] * ort[i];
      }
      double g = sqrt(h);
      if (ort[m] > 0) {
	g = -g;
      }
      h = h - ort[m] * g;
      ort[m] = ort[m] - g;
      
      // Apply Householder similarity transformation
      // H = (I-u*u'/h)*H*(I-u*u')/h)
      
      for (uint j = m; j < _n; j++) {
	double f = 0.0;
	for (uint i = high; i >= m; i--) {
	  f += ort[i]*H[i][j];
	}
	f = f/h;
	for (uint i = m; i <= high; i++) {
	  H[i][j] -= f*ort[i];
	}
      }
      
      for (uint i = 0; i <= high; i++) {
	double f = 0.0;
	for (uint j = high; j >= m; j--) {
	  f += ort[j]*H[i][j];
	}
	f = f/h;
	for (uint j = m; j <= high; j++) {
	  H[i][j] -= f*ort[j];
	}
      }
      ort[m] = scale*ort[m];
      H[m][m-1] = scale*g;
    }
  }
  
  // Accumulate transformations (Algol's ortran).
  
  for (uint i = 0; i < _n; i++) {
    for (uint j = 0; j < _n; j++) {
      V[i][j] = (i == j ? 1.0 : 0.0);
    }
  }
  
  for (uint m = high-1; m >= low+1; m--) {
    if (H[m][m-1] != 0.0) {
      for (uint i = m+1; i <= high; i++) {
	ort[i] = H[i][m-1];
      }
      for (uint j = m; j <= high; j++) {
	double g = 0.0;
	for (uint i = m; i <= high; i++) {
	  g += ort[i] * V[i][j];
	}
	// Double division avoids possible underflow
	g = (g / ort[m]) / H[m][m-1];
	for (uint i = m; i <= high; i++) {
	  V[i][j] += g * ort[i];
	}
      }
    }
  }
}

void Matrix::hqr2 () {
  
  //  This is derived from the Algol procedure hqr2,
  //  by Martin and Wilkinson, Handbook for Auto. Comp.,
  //  Vol.ii-Linear Algebra, and the corresponding
  //  Fortran subroutine in EISPACK.
  
  // Initialize
  
  int nn = (int)_n;
  int n = nn-1;
  int low = 0;
  int high = nn-1;
  double eps = pow(2.0,-52.0);
  double exshift = 0.0;
  double p=0,q=0,r=0,s=0,z=0,t,w,x,y;
  
  // Store roots isolated by balanc and compute matrix norm
  
  double norm = 0.0;
  for (int i = 0; i < nn; i++) {
    if ((i < low) || (i > high)) {
      d[i] = H[i][i];
      e[i] = 0.0;
    }
    for (int j = max(i-1,0); j < nn; j++) {
      norm = norm + fabs(H[i][j]);
    }
  }
  
  // Outer loop over eigenvalue index
  
  int iter = 0;
  while (n >= low) {
    
    // Look for single small sub-diagonal element
    
    int l = n;
    while (l > low) {
      s = fabs(H[l-1][l-1]) + fabs(H[l][l]);
      if (s == 0.0) {
	s = norm;
      }
      if (fabs(H[l][l-1]) < eps * s) {
	break;
      }
      l--;
    }
    
    // Check for convergence
    // One root found
    
    if (l == n) {
      H[n][n] = H[n][n] + exshift;
      d[n] = H[n][n];
      e[n] = 0.0;
      n--;
      iter = 0;
      
      // Two roots found
      
    } else if (l == n-1) {
      w = H[n][n-1] * H[n-1][n];
      p = (H[n-1][n-1] - H[n][n]) / 2.0;
      q = p * p + w;
      z = sqrt(fabs(q));
      H[n][n] = H[n][n] + exshift;
      H[n-1][n-1] = H[n-1][n-1] + exshift;
      x = H[n][n];
      
      // double pair
      
      if (q >= 0) {
	if (p >= 0) {
	  z = p + z;
	} else {
	  z = p - z;
	}
	d[n-1] = x + z;
	d[n] = d[n-1];
	if (z != 0.0) {
	  d[n] = x - w / z;
	}
	e[n-1] = 0.0;
	e[n] = 0.0;
	x = H[n][n-1];
	s = fabs(x) + fabs(z);
	p = x / s;
	q = z / s;
	r = sqrt(p * p+q * q);
	p = p / r;
	q = q / r;
	
	// Row modification
	
	for (int j = n-1; j < nn; j++) {
	  z = H[n-1][j];
	  H[n-1][j] = q * z + p * H[n][j];
	  H[n][j] = q * H[n][j] - p * z;
	}
	
	// Column modification
	
	for (int i = 0; i <= n; i++) {
	  z = H[i][n-1];
	  H[i][n-1] = q * z + p * H[i][n];
	  H[i][n] = q * H[i][n] - p * z;
	}
	
	// Accumulate transformations
	
	for (int i = low; i <= high; i++) {
	  z = V[i][n-1];
	  V[i][n-1] = q * z + p * V[i][n];
	  V[i][n] = q * V[i][n] - p * z;
	}
	
	// Complex pair
	
      } else {
	d[n-1] = x + p;
	d[n] = x + p;
	e[n-1] = z;
	e[n] = -z;
      }
      n = n - 2;
      iter = 0;
      
      // No convergence yet
      
    } else {
      
      // Form shift
      
      x = H[n][n];
      y = 0.0;
      w = 0.0;
      if (l < n) {
	y = H[n-1][n-1];
	w = H[n][n-1] * H[n-1][n];
      }
      
      // Wilkinson's original ad hoc shift
      
      if (iter == 10) {
	exshift += x;
	for (int i = low; i <= n; i++) {
	  H[i][i] -= x;
	}
	s = fabs(H[n][n-1]) + fabs(H[n-1][n-2]);
	x = y = 0.75 * s;
	w = -0.4375 * s * s;
      }
      
      // MATLAB's new ad hoc shift
      
      if (iter == 30) {
	s = (y - x) / 2.0;
	s = s * s + w;
	if (s > 0) {
	  s = sqrt(s);
	  if (y < x) {
	    s = -s;
	  }
	  s = x - w / ((y - x) / 2.0 + s);
	  for (int i = low; i <= n; i++) {
	    H[i][i] -= s;
	  }
	  exshift += s;
	  x = y = w = 0.964;
	}
      }
      
      iter = iter + 1;   // (Could check iteration count here.)
      
      // Look for two consecutive small sub-diagonal elements
      
      int m = n-2;
      while (m >= l) {
	z = H[m][m];
	r = x - z;
	s = y - z;
	p = (r * s - w) / H[m+1][m] + H[m][m+1];
	q = H[m+1][m+1] - z - r - s;
	r = H[m+2][m+1];
	s = fabs(p) + fabs(q) + fabs(r);
	p = p / s;
	q = q / s;
	r = r / s;
	if (m == l) {
	  break;
	}
	if (fabs(H[m][m-1]) * (fabs(q) + fabs(r)) <
	    eps * (fabs(p) * (fabs(H[m-1][m-1]) + fabs(z) +
			     fabs(H[m+1][m+1])))) {
	  break;
	}
	m--;
      }
      
      for (int i = m+2; i <= n; i++) {
	H[i][i-2] = 0.0;
	if (i > m+2) {
	  H[i][i-3] = 0.0;
	}
      }
      
      // Double QR step involving rows l:n and columns m:n
      
      for (int k = m; k <= n-1; k++) {
	int notlast = (k != n-1);
	if (k != m) {
	  p = H[k][k-1];
	  q = H[k+1][k-1];
	  r = (notlast ? H[k+2][k-1] : 0.0);
	  x = fabs(p) + fabs(q) + fabs(r);
	  if (x != 0.0) {
	    p = p / x;
	    q = q / x;
	    r = r / x;
	  }
	}
	if (x == 0.0) {
	  break;
	}
	s = sqrt(p * p + q * q + r * r);
	if (p < 0) {
	  s = -s;
	}
	if (s != 0) {
	  if (k != m) {
	    H[k][k-1] = -s * x;
	  } else if (l != m) {
	    H[k][k-1] = -H[k][k-1];
	  }
	  p = p + s;
	  x = p / s;
	  y = q / s;
	  z = r / s;
	  q = q / p;
	  r = r / p;
	  
	  // Row modification
	  
	  for (int j = k; j < nn; j++) {
	    p = H[k][j] + q * H[k+1][j];
	    if (notlast) {
	      p = p + r * H[k+2][j];
	      H[k+2][j] = H[k+2][j] - p * z;
	    }
	    H[k][j] = H[k][j] - p * x;
	    H[k+1][j] = H[k+1][j] - p * y;
	  }
	  
	  // Column modification
	  
	  for (int i = 0; i <= min(n,k+3); i++) {
	    p = x * H[i][k] + y * H[i][k+1];
	    if (notlast) {
	      p = p + z * H[i][k+2];
	      H[i][k+2] = H[i][k+2] - p * r;
	    }
	    H[i][k] = H[i][k] - p;
	    H[i][k+1] = H[i][k+1] - p * q;
	  }
	  
	  // Accumulate transformations
	  
	  for (int i = low; i <= high; i++) {
	    p = x * V[i][k] + y * V[i][k+1];
	    if (notlast) {
	      p = p + z * V[i][k+2];
	      V[i][k+2] = V[i][k+2] - p * r;
	    }
	    V[i][k] = V[i][k] - p;
	    V[i][k+1] = V[i][k+1] - p * q;
	  }
	}  // (s != 0)
      }  // k loop
    }  // check convergence
  }  // while (n >= low)
  
  // Backsubstitute to find vectors of upper triangular form
  
  if (norm == 0.0) {
    return;
  }
  
  for (n = nn-1; n >= 0; n--) {
    p = d[n];
    q = e[n];
    
    // double vector
    
    if (q == 0) {
      int l = n;
      H[n][n] = 1.0;
      for (int i = n-1; i >= 0; i--) {
	w = H[i][i] - p;
	r = 0.0;
	for (int j = l; j <= n; j++) {
	  r = r + H[i][j] * H[j][n];
	}
	if (e[i] < 0.0) {
	  z = w;
	  s = r;
	} else {
	  l = i;
	  if (e[i] == 0.0) {
	    if (w != 0.0) {
	      H[i][n] = -r / w;
	    } else {
	      H[i][n] = -r / (eps * norm);
	    }
	    
	    // Solve real equations
	    
	  } else {
	    x = H[i][i+1];
	    y = H[i+1][i];
	    q = (d[i] - p) * (d[i] - p) + e[i] * e[i];
	    t = (x * s - z * r) / q;
	    H[i][n] = t;
	    if (fabs(x) > fabs(z)) {
	      H[i+1][n] = (-r - w * t) / x;
	    } else {
	      H[i+1][n] = (-s - y * t) / z;
	    }
	  }
	  
	  // Overflow control
	  
	  t = fabs(H[i][n]);
	  if ((eps * t) * t > 1) {
	    for (int j = i; j <= n; j++) {
	      H[j][n] = H[j][n] / t;
	    }
	  }
	}
      }
      
      // Complex vector
      
    } else if (q < 0) {
      int l = n-1;
      
      // Last vector component imaginary so matrix is triangular
      
      if (fabs(H[n][n-1]) > fabs(H[n-1][n])) {
	H[n-1][n-1] = q / H[n][n-1];
	H[n-1][n] = -(H[n][n] - p) / H[n][n-1];
      } else {
	cdiv(0.0, -H[n-1][n], H[n-1][n-1]-p, q, H[n-1][n-1], H[n-1][n]);
      }
      H[n][n-1] = 0.0;
      H[n][n] = 1.0;
      for (int i = n-2; i >= 0; i--) {
	double ra,sa,vr,vi;
	ra = 0.0;
	sa = 0.0;
	for (int j = l; j <= n; j++) {
	  ra = ra + H[i][j] * H[j][n-1];
	  sa = sa + H[i][j] * H[j][n];
	}
	w = H[i][i] - p;
	
	if (e[i] < 0.0) {
	  z = w;
	  r = ra;
	  s = sa;
	} else {
	  l = i;
	  if (e[i] == 0) {
	    cdiv(-ra, -sa, w, q, H[i][n-1], H[i][n]);
	  } else {
	    
	    // Solve complex equations
	    
	    x = H[i][i+1];
	    y = H[i+1][i];
	    vr = (d[i] - p) * (d[i] - p) + e[i] * e[i] - q * q;
	    vi = (d[i] - p) * 2.0 * q;
	    if ((vr == 0.0) && (vi == 0.0)) {
	      vr = eps * norm * (fabs(w) + fabs(q) +
				 fabs(x) + fabs(y) + fabs(z));
	    }
	    cdiv(x*r-z*ra+q*sa, x*s-z*sa-q*ra, vr, vi, H[i][n-1], H[i][n]);
	    if (fabs(x) > (fabs(z) + fabs(q))) {
	      H[i+1][n-1] = (-ra - w * H[i][n-1] + q * H[i][n]) / x;
	      H[i+1][n] = (-sa - w * H[i][n] - q * H[i][n-1]) / x;
	    } else {
	      cdiv(-r-y*H[i][n-1], -s-y*H[i][n], z, q, H[i+1][n-1], H[i+1][n]);
	    }
	  }
	  
	  // Overflow control
	  
	  t = max(fabs(H[i][n-1]),fabs(H[i][n]));
	  if ((eps * t) * t > 1) {
	    for (int j = i; j <= n; j++) {
	      H[j][n-1] = H[j][n-1] / t;
	      H[j][n] = H[j][n] / t;
	    }
	  }
	}
      }
    }
  }
  
  // Vectors of isolated roots
  
  for (int i = 0; i < nn; i++) {
    if (i < low || i > high) {
      for (int j = i; j < nn; j++) {
	V[i][j] = H[i][j];
      }
    }
  }
  
  // Back transformation to get eigenvectors of original matrix
  
  for (int j = nn-1; j >= low; j--) {
    for (int i = low; i <= high; i++) {
      z = 0.0;
      for (int k = low; k <= min(j,high); k++) {
	z = z + V[i][k] * H[k][j];
      }
      V[i][j] = z;
    }
  }
}
