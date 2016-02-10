#ifndef MATRIX_HH
#define MATRIX_HH

#include <fstream>
#include <stdlib.h>
#include <sys/time.h>

#include <math.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_eigen.h>

#include <macros.h>
#include <Vector.hh>

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif


typedef enum {

  DIAGONAL, SYMMETRIC, U_TRIANGULAR, GENERIC

} MatrixType;

class Matrix
{
public:

  // Constructors
  Matrix();
  explicit Matrix(uint m, MatrixType type = GENERIC);
  explicit Matrix(uint m, uint n, MatrixType type = GENERIC);
  explicit Matrix(double *data, uint m, MatrixType type = GENERIC);
  explicit Matrix(double *data, uint m, uint n, MatrixType type = GENERIC);
  Matrix(Matrix const &other);

  // Destructor
  ~Matrix();

  // Reading functions
  uint Rows() const { return _m; };
  uint Cols() const { return _n; };
  uint Size() const { return _size; };
  MatrixType Type() const {  return _type; };
  Vector Row(uint i) const throw (Exception *);
  Vector Col(uint i) const throw (Exception *);
  double Max() const;
  double Min() const;
  void Write(const char *filename) const throw (Exception *);
  void ExportPGM(const char *filename) const throw (Exception *);

  // Writing functions
  void Read(const char *filename) throw (Exception *);

  // Operators
  double *operator[](uint m) throw (Exception *);
  const double *operator[](uint m) const throw (Exception *);
  Matrix &operator=(Matrix const &other);

  Matrix  operator+ (double x) const;
  Matrix  operator+ (Matrix const &other) const throw (Exception *);
  Matrix& operator+=(double x);
  Matrix& operator+=(Matrix const &other) throw (Exception *);
  Matrix  operator- (double x) const;
  Matrix  operator- (Matrix const &other) const throw (Exception *);
  Matrix& operator-=(double x);
  Matrix& operator-=(Matrix const &other) throw (Exception *);
  Matrix  operator- () const;
  Matrix  operator* (double x) const;
  Matrix  operator* (Matrix const &other) const throw (Exception *);
  Vector  operator* (Vector const &vec) const throw (Exception *);
  Matrix& operator*=(double x);
  Matrix  operator/ (double x) const throw (Exception *);
  Matrix& operator/= (double x) throw (Exception *);
  friend Vector operator* (Vector const &vec, Matrix const &mat) 
    throw (Exception *);
  friend Matrix operator* (double x, Matrix const &mat);
  friend Matrix operator/ (double x, Matrix const &mat) throw (Exception *);

  // Concatenate matrices and vectors
  Matrix operator|(Vector const &vec) const throw (Exception *);
  Matrix operator|=(Vector const &vec) throw (Exception *);
  Matrix operator|(Matrix const &other) const throw (Exception *);
  Matrix operator|=(Matrix const &other) throw (Exception *);

  // Stack matrices and vectors
  Matrix operator,(Vector const &vec) const throw (Exception *);
  Matrix operator,(Matrix const &other) const throw (Exception *);


  // Member functions
  Matrix Submat(uint i1, uint j1, uint i2, uint j2) const throw (Exception *);
  Matrix Flip(char const *dir); // Either "ROWS" or "COLS"
  Matrix Transp() const;
  Matrix TranspTimes(Matrix const & other) const throw (Exception *);
  Matrix TimesTransp(Matrix const & other) const throw (Exception *);
  double ElemMult(Matrix const &other) const throw (Exception *);
  double Fnorm() const;
  Matrix Invert() const throw (Exception *);
  Matrix Pinv() const throw (Exception *);
  void QRFac(Matrix &Q, Matrix &R) const throw (Exception *);
  void SVD(Matrix &U, Matrix &D, Matrix &VT) const throw (Exception *);
  void Chol(Matrix &L) const throw (Exception *);
  double Det() const throw (Exception *);
  double Sum() const;
  double Trace() const;
  Matrix Convolve(Matrix const &kernel) const throw (Exception *);
  double Interpolate(double x, double y) const throw (Exception *);
  Vector Mean(int dim) const;
  Matrix AutoCov(int dim) const;
  Matrix CrossCov(Matrix const &other, int dim) const;
  void EigenSolve(Vector &eval, Matrix &evec, bool real = true) throw (Exception*);
  Matrix Translate(Vector const &vec, char const *dir) throw (Exception *); // Either "ROW" or "COL"

  static Matrix Diag(double *data, uint len);
  static Matrix Identity(uint m, uint n);
  static Matrix Random(uint m, uint n);
  static Matrix Ones(uint m, uint n);
  static Matrix Zeros(uint m, uint n);
  static Matrix OutProd(Vector const &v);
  static Matrix OutProd(Vector const &v, Vector const &w);
  static void EigenSolveSymm(uint m, Vector const &data,
                             Vector &eval, Matrix &evec, bool sort = true);
  static void EigenSolveSymm(uint m, double *data,
			     double *eval, double *evec, bool sort = true);

protected:
  void copy(Matrix const &other);
  void destroy();
  uint _m;
  uint _n;
  uint _size;
  double *_data;
  MatrixType _type;

private:


  double **H, **V;
  double *ort;
  double *d, *e;
  bool eigen_vec_computed;

  // complex division
  void cdiv(double xr, double xi, double yr, double yi, double &cdivr, double &cdivi);

  // Nonsymmetric reduction to Hessenberg form.
  void orthes ();

 // Nonsymmetric reduction from Hessenberg to real Schur form.
  void hqr2 ();

};

ostream &operator<<(ostream &stream, Matrix const &mat);

Matrix operator|(Vector const &vec1, Vector const &vec2) throw (Exception *);
Matrix operator|(Vector const &vec, Matrix const &mat) throw (Exception *);

#endif
