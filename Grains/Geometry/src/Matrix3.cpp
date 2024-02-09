#include "Matrix3.hh"


// -----------------------------------------------------------------------------
// Default constructor. Matrix is initialized to the identity matrix
template <typename T>
__host__ __device__ Matrix3<T>::Matrix3()
{
  setValue( T( 1 ), T( 0 ), T( 0 ), 
            T( 0 ), T( 1 ), T( 0 ), 
            T( 0 ), T( 0 ), T( 1 ) );
}




// -----------------------------------------------------------------------------
// Constructor with a 1D array of values as input
template <typename T>
__host__ __device__ Matrix3<T>::Matrix3( T const* mat )
{
  setValue( mat );
}




// -----------------------------------------------------------------------------
// Constructor with 9 components as inputs
template <typename T>
__host__ __device__ Matrix3<T>::Matrix3( T xx, T xy, T xz,
                                         T yx, T yy, T yz,
                                         T zx, T zy, T zz )
{
  setValue( xx, xy, xz, yx, yy, yz, zx, zy, zz );
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__host__ __device__ Matrix3<T>::Matrix3( Matrix3<T> const& mat )
{
  for ( int i = 0; i < 3; ++i )
    for ( int j = 0; j < 3; ++j )
      m_comp[i][j] = mat.m_comp[i][j];
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__ Matrix3<T>::~Matrix3()
{}




// -----------------------------------------------------------------------------
// Sets the matrix to a 1D array of 9 values as input
template <typename T>
__host__ __device__ void Matrix3<T>::setValue( T const* mat )
{
  m_comp[X][X] = *mat++; m_comp[X][Y] = *mat++; m_comp[X][Z] = *mat++;
  m_comp[Y][X] = *mat++; m_comp[Y][Y] = *mat++; m_comp[Y][Z] = *mat++;
  m_comp[Z][X] = *mat++; m_comp[Z][Y] = *mat++; m_comp[Z][Z] = *mat;
}




// -----------------------------------------------------------------------------
// Sets the matrix with all 9 components as inputs
template <typename T>
__host__ __device__ void Matrix3<T>::setValue( T xx, T xy, T xz,
                                               T yx, T yy, T yz,
                                               T zx, T zy, T zz )
{
  m_comp[X][X] = xx; m_comp[X][Y] = xy; m_comp[X][Z] = xz;
  m_comp[Y][X] = yx; m_comp[Y][Y] = yy; m_comp[Y][Z] = yz;
  m_comp[Z][X] = zx; m_comp[Z][Y] = zy; m_comp[Z][Z] = zz;
}




// -----------------------------------------------------------------------------
// Returns a matrix with positive components
template <typename T>
__host__ __device__ Matrix3<T> Matrix3<T>::absolute() const
{
  return ( Matrix3<T>(
  	 fabs( m_comp[X][X] ), fabs( m_comp[X][Y] ), fabs( m_comp[X][Z] ),
	   fabs( m_comp[Y][X] ), fabs( m_comp[Y][Y] ), fabs( m_comp[Y][Z] ),
	   fabs( m_comp[Z][X] ), fabs( m_comp[Z][Y] ), fabs( m_comp[Z][Z] ) ) );
}




// -----------------------------------------------------------------------------
// Returns a matrix with positive components - specialized for floats
template <>
__host__ __device__ Matrix3<float> Matrix3<float>::absolute() const
{
  return ( Matrix3<float>(
  	 fabsf( m_comp[X][X] ), fabsf( m_comp[X][Y] ), fabsf( m_comp[X][Z] ),
	   fabsf( m_comp[Y][X] ), fabsf( m_comp[Y][Y] ), fabsf( m_comp[Y][Z] ),
	   fabsf( m_comp[Z][X] ), fabsf( m_comp[Z][Y] ), fabsf( m_comp[Z][Z] ) ) );
}




// -----------------------------------------------------------------------------
// Returns the determinant of the matrix
template <typename T>
__host__ __device__ T Matrix3<T>::determinant() const
{
  return ( 
    m_comp[X][X] * ( m_comp[Y][Y] * m_comp[Z][Z] - m_comp[Y][Z] * m_comp[Z][Y] )
  - m_comp[X][Y] * ( m_comp[Y][X] * m_comp[Z][Z] - m_comp[Y][Z] * m_comp[Z][X] )
  + m_comp[X][Z] * ( m_comp[Y][X] * m_comp[Z][Y] - m_comp[Y][Y] * m_comp[Z][X] )
  );
}




// -----------------------------------------------------------------------------
// Returns the inverse of the matrix
template <typename T>
__host__ __device__ Matrix3<T> Matrix3<T>::inverse() const
{
  Vector3<T> co( m_comp[Y][Y] * m_comp[Z][Z] - m_comp[Y][Z] * m_comp[Z][Y],
	               m_comp[Y][Z] * m_comp[Z][X] - m_comp[Y][X] * m_comp[Z][Z],
	               m_comp[Y][X] * m_comp[Z][Y] - m_comp[Y][Y] * m_comp[Z][X] );
  T d = (*this)[X] * co;
//  assert( !eqz( d ) ); EPSILON = 1.e-10
  T s = T( 1 ) / d;
  return ( Matrix3<T>( co[X] * s,
	( m_comp[X][Z] * m_comp[Z][Y] - m_comp[X][Y] * m_comp[Z][Z] ) * s,
	( m_comp[X][Y] * m_comp[Y][Z] - m_comp[X][Z] * m_comp[Y][Y] ) * s,
	co[Y] * s,
	( m_comp[X][X] * m_comp[Z][Z] - m_comp[X][Z] * m_comp[Z][X] ) * s,
	( m_comp[X][Z] * m_comp[Y][X] - m_comp[X][X] * m_comp[Y][Z] ) * s,
	co[Z] * s,
	( m_comp[X][Y] * m_comp[Z][X] - m_comp[X][X] * m_comp[Z][Y] ) * s,
	( m_comp[X][X] * m_comp[Y][Y] - m_comp[X][Y] * m_comp[Y][X] ) * s ) );
}




// -----------------------------------------------------------------------------
// Returns the transposed matrix
template <typename T>
__host__ __device__ Matrix3<T> Matrix3<T>::transpose() const
{
  return ( Matrix3<T>( m_comp[X][X], m_comp[Y][X], m_comp[Z][X],
	                     m_comp[X][Y], m_comp[Y][Y], m_comp[Z][Y],
	                     m_comp[X][Z], m_comp[Y][Z], m_comp[Z][Z] ) );
}




// -----------------------------------------------------------------------------
// Operator +=
template <typename T>
__host__ __device__ Matrix3<T>& Matrix3<T>::operator += ( Matrix3<T> const& m )
{
  setValue(
  	m_comp[X][X] + m[X][X], m_comp[X][Y] + m[X][Y], m_comp[X][Z] + m[X][Z],
	  m_comp[Y][X] + m[Y][X], m_comp[Y][Y] + m[Y][Y], m_comp[Y][Z] + m[Y][Z],
	  m_comp[Z][X] + m[Z][X], m_comp[Z][Y] + m[Z][Y], m_comp[Z][Z] + m[Z][Z] );
  return ( *this );
}




// -----------------------------------------------------------------------------
// Operator -=
template <typename T>
__host__ __device__ Matrix3<T>& Matrix3<T>::operator -= ( Matrix3<T> const& m )
{
  setValue(
  	m_comp[X][X] - m[X][X], m_comp[X][Y] - m[X][Y], m_comp[X][Z] - m[X][Z],
	  m_comp[Y][X] - m[Y][X], m_comp[Y][Y] - m[Y][Y], m_comp[Y][Z] - m[Y][Z],
	  m_comp[Z][X] - m[Z][X], m_comp[Z][Y] - m[Z][Y], m_comp[Z][Z] - m[Z][Z] );
  return ( *this );
}




// -----------------------------------------------------------------------------
// Operator *= by a scalar
template <typename T>
__host__ __device__ Matrix3<T>& Matrix3<T>::operator *= ( T d )
{
  setValue(
  	d * m_comp[X][X], d * m_comp[X][Y], d * m_comp[X][Z],
	  d * m_comp[Y][X], d * m_comp[Y][Y], d * m_comp[Y][Z],
	  d * m_comp[Z][X], d * m_comp[Z][Y], d * m_comp[Z][Z] );
  return ( *this );
}




// -----------------------------------------------------------------------------
// Operator *= by a matrix
template <typename T>
__host__ __device__ Matrix3<T>& Matrix3<T>::operator *= ( Matrix3<T> const& m )
{
  setValue(
    m_comp[X][X] * m[X][X] + m_comp[X][Y] * m[Y][X] + m_comp[X][Z] * m[Z][X],
    m_comp[X][X] * m[X][Y] + m_comp[X][Y] * m[Y][Y] + m_comp[X][Z] * m[Z][Y],
    m_comp[X][X] * m[X][Z] + m_comp[X][Y] * m[Y][Z] + m_comp[X][Z] * m[Z][Z],
    m_comp[Y][X] * m[X][X] + m_comp[Y][Y] * m[Y][X] + m_comp[Y][Z] * m[Z][X],
    m_comp[Y][X] * m[X][Y] + m_comp[Y][Y] * m[Y][Y] + m_comp[Y][Z] * m[Z][Y],
    m_comp[Y][X] * m[X][Z] + m_comp[Y][Y] * m[Y][Z] + m_comp[Y][Z] * m[Z][Z],
    m_comp[Z][X] * m[X][X] + m_comp[Z][Y] * m[Y][X] + m_comp[Z][Z] * m[Z][X],
    m_comp[Z][X] * m[X][Y] + m_comp[Z][Y] * m[Y][Y] + m_comp[Z][Z] * m[Z][Y],
    m_comp[Z][X] * m[X][Z] + m_comp[Z][Y] * m[Y][Z] + m_comp[Z][Z] * m[Z][Z] );
  return ( *this );
}




// -----------------------------------------------------------------------------
// i-th row accessor
template <typename T>
__host__ __device__ Vector3<T>& Matrix3<T>::operator [] ( unsigned int i ) const
{
  return ( *( Vector3<T>* )m_comp[i] );
}




// -----------------------------------------------------------------------------
// Assign operator to another matrix
template <typename T>
__host__ __device__ Matrix3<T>& Matrix3<T>::operator = ( Matrix3<T> const& m )
{
  if ( &m != this )
  {
    for ( int i = 0; i < 3; ++i )
      for ( int j = 0; j < 3; ++j )
        m_comp[i][j] = m.m_comp[i][j];
  }
  return ( *this );
}




// // -----------------------------------------------------------------------------
// // Unitary operator -. Returns an object with negative components
// template <typename T>
// __host__ __device__ Matrix3<T> Matrix3<T>::operator - () const
// {
//   setValue(
//     - m_comp[X][X], - m_comp[X][Y], - m_comp[X][Z],
// 	  - m_comp[Y][X], - m_comp[Y][Y], - m_comp[Y][Z],
// 	  - m_comp[Z][X], - m_comp[Z][Y], - m_comp[Z][Z] );
//   return ( *this );
// }




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Matrices sum
template <typename T>
__host__ __device__ Matrix3<T> operator + ( Matrix3<T> const& m1,
                                            Matrix3<T> const& m2 )
{
  return (
    Matrix3<T>( m1[X][X] + m2[X][X], m1[X][Y] + m2[X][Y], m1[X][Z] + m2[X][Z],
                m1[Y][X] + m2[Y][X], m1[Y][Y] + m2[Y][Y], m1[Y][Z] + m2[Y][Z],
                m1[Z][X] + m2[Z][X], m1[Z][Y] + m2[Z][Y], m1[Z][Z] + m2[Z][Z] ) );
}




// -----------------------------------------------------------------------------
// Multiplies a matrix by a scalar
template <typename T>
__host__ __device__ Matrix3<T> operator * ( T c, Matrix3<T> const& m )
{
  return ( Matrix3<T>( c * m[X][X], c * m[X][Y], c * m[X][Z],
                       c * m[Y][X], c * m[Y][Y], c * m[Y][Z],
                       c * m[Z][X], c * m[Z][Y], c * m[Z][Z] ) );
}




// -----------------------------------------------------------------------------
// Matrix-vector product
template <typename T>
__host__ __device__ Vector3<T> operator * ( Matrix3<T> const& m, 
                                            Vector3<T> const& v )
{
  return ( Vector3<T>( m[X] * v, m[Y] * v, m[Z] * v ) );
}




// ----------------------------------------------------------------------------
// Vector-matrix product
template <typename T>
__host__ __device__ Vector3<T> operator * ( Vector3<T> const& v, 
                                            Matrix3<T> const& m )
{
  Matrix3<T> m_tr = m.transpose();
  return ( Vector3<T>( m_tr[X] * v, m_tr[Y] * v, m_tr[Z] * v ) );
}




// ----------------------------------------------------------------------------
// Matrix-matrix product
template <typename T>
__host__ __device__ Matrix3<T> operator * ( Matrix3<T> const& m1, 
                                            Matrix3<T> const& m2 )
{
  return Matrix3<T>(
    m1[X][X] * m2[X][X] + m1[X][Y] * m2[Y][X] + m1[X][Z] * m2[Z][X],
    m1[X][X] * m2[X][Y] + m1[X][Y] * m2[Y][Y] + m1[X][Z] * m2[Z][Y],
    m1[X][X] * m2[X][Z] + m1[X][Y] * m2[Y][Z] + m1[X][Z] * m2[Z][Z],
    m1[Y][X] * m2[X][X] + m1[Y][Y] * m2[Y][X] + m1[Y][Z] * m2[Z][X],
    m1[Y][X] * m2[X][Y] + m1[Y][Y] * m2[Y][Y] + m1[Y][Z] * m2[Z][Y],
    m1[Y][X] * m2[X][Z] + m1[Y][Y] * m2[Y][Z] + m1[Y][Z] * m2[Z][Z],
    m1[Z][X] * m2[X][X] + m1[Z][Y] * m2[Y][X] + m1[Z][Z] * m2[Z][X],
    m1[Z][X] * m2[X][Y] + m1[Z][Y] * m2[Y][Y] + m1[Z][Z] * m2[Z][Y],
    m1[Z][X] * m2[X][Z] + m1[Z][Y] * m2[Y][Z] + m1[Z][Z] * m2[Z][Z] );
}




// // ----------------------------------------------------------------------------
// // Returns the matrix rotates vector src to vector dest, i.e. dest = mat * src
// Matrix getRotationMatrix( Vector3 const& src, Vector3 const& dest )
// {
//   // double c = src * dest;
//   // if ( fabs( c + 1. ) < EPSILON )
//   //   return( Matrix( -1.,  0.,  0.,
//   //                    0., -1.,  0.,
//   //                    0.,  0., -1.) );
//   // Vector3 v = src ^ dest;
//   // Matrix skewV  = Matrix(    0., -v[Z],  v[Y],
//   //                          v[Z],    0., -v[X],
//   //                         -v[Y],  v[X],    0.);
//   //
//   // Matrix rotMat = Matrix( 1., 1., 1. ) + skewV +
//   //                 ( 1. / ( 1. + c ) ) * ( skewV * skewV );
//   // rotMat.round( EPSILON );

//   // More efficient approach
//   double c = src * dest;
//   if ( fabs( c + 1. ) < sqrt(EPSILON) )
//     return( Matrix( -1.,  0.,  0.,
//                      0., -1.,  0.,
//                      0.,  0., -1.) );
//   double den = 1. / ( 1. + c );
//   Vector3 v = src ^ dest;
//   Matrix rotMat = Matrix( 1. - (v[Y]*v[Y] + v[Z]*v[Z]) * den,
//                           -v[Z] + v[X]*v[Y] * den,
//                           v[Y] + v[X]*v[Z] * den,
//                           v[Z] + v[X]*v[Y] * den,
//                           1. - (v[X]*v[X] + v[Z]*v[Z]) * den,
//                           -v[X] + v[Y]*v[Z] * den,
//                           -v[Y] + v[X]*v[Z] * den,
//                           v[X] + v[Y]*v[Z] * den,
//                           1. - (v[X]*v[X] + v[Y]*v[Y]) * den );
//   rotMat.round( EPSILON );
//   return ( rotMat );
// }




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Matrix3<float>;
template class Matrix3<double>;

#define X( T ) \
template Matrix3<T> operator +<T>( Matrix3<T> const& m1, Matrix3<T> const& m2 );\
template Matrix3<T> operator *<T>( T c, Matrix3<T> const& m );\
template Vector3<T> operator *<T>( Matrix3<T> const& m, Vector3<T> const& v );\
template Vector3<T> operator *<T>( Vector3<T> const& v, Matrix3<T> const& m );\
template Matrix3<T> operator *<T>( Matrix3<T> const& m1, Matrix3<T> const& m2 );
X( float )
X( double )
#undef X






