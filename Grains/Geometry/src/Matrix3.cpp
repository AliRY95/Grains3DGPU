#include "Matrix3.hh"
#include "VectorMath.hh"


// -----------------------------------------------------------------------------
// Default constructor. Matrix is initialized to the identity matrix
template <typename T>
__HOSTDEVICE__
Matrix3<T>::Matrix3()
{
    setValue( T( 1 ), T( 0 ), T( 0 ), 
              T( 0 ), T( 1 ), T( 0 ), 
              T( 0 ), T( 0 ), T( 1 ) );
}




// -----------------------------------------------------------------------------
// Constructor with a 1D array of values as input
template <typename T>
__HOSTDEVICE__
Matrix3<T>::Matrix3( T const* mat )
{
    setValue( mat );
}




// -----------------------------------------------------------------------------
// Constructor with 9 components as inputs
template <typename T>
__HOSTDEVICE__
Matrix3<T>::Matrix3( T xx, T xy, T xz,
                     T yx, T yy, T yz,
                     T zx, T zy, T zz )
{
    setValue( xx, xy, xz, yx, yy, yz, zx, zy, zz );
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__
Matrix3<T>::Matrix3( Matrix3<T> const& mat )
{
  for ( int i = 0; i < 3; ++i )
    for ( int j = 0; j < 3; ++j )
      m_comp[i][j] = mat.m_comp[i][j];
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Matrix3<T>::~Matrix3()
{}




// -----------------------------------------------------------------------------
// Sets the matrix to a 1D array of 9 values as input
template <typename T>
__HOSTDEVICE__
void Matrix3<T>::setValue( T const* mat )
{
    m_comp[X][X] = *mat++; m_comp[X][Y] = *mat++; m_comp[X][Z] = *mat++;
    m_comp[Y][X] = *mat++; m_comp[Y][Y] = *mat++; m_comp[Y][Z] = *mat++;
    m_comp[Z][X] = *mat++; m_comp[Z][Y] = *mat++; m_comp[Z][Z] = *mat;
}




// -----------------------------------------------------------------------------
// Sets the matrix with all 9 components as inputs
template <typename T>
__HOSTDEVICE__
void Matrix3<T>::setValue( T xx, T xy, T xz,
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
__HOSTDEVICE__
Matrix3<T> Matrix3<T>::absolute() const
{
    return ( Matrix3<T>(
        fabs( m_comp[X][X] ), fabs( m_comp[X][Y] ), fabs( m_comp[X][Z] ),
        fabs( m_comp[Y][X] ), fabs( m_comp[Y][Y] ), fabs( m_comp[Y][Z] ),
        fabs( m_comp[Z][X] ), fabs( m_comp[Z][Y] ), fabs( m_comp[Z][Z] ) ) );
}




// -----------------------------------------------------------------------------
// Returns the determinant of the matrix
template <typename T>
__HOSTDEVICE__
T Matrix3<T>::determinant() const
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
__HOSTDEVICE__
Matrix3<T> Matrix3<T>::inverse() const
{
    Vector3<T> co( m_comp[Y][Y] * m_comp[Z][Z] - m_comp[Y][Z] * m_comp[Z][Y],
                   m_comp[Y][Z] * m_comp[Z][X] - m_comp[Y][X] * m_comp[Z][Z],
                   m_comp[Y][X] * m_comp[Z][Y] - m_comp[Y][Y] * m_comp[Z][X] );
    T d = (*this)[X] * co;
    if ( fabs( d ) < HIGHEPS<T> )
        printf( "Matrix is not inversible!\n" );
    T s = T( 1 ) / d;
    return ( Matrix3<T>( 
    co[X] * s,
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
__HOSTDEVICE__
Matrix3<T> Matrix3<T>::transpose() const
{
    return ( Matrix3<T>( m_comp[X][X], m_comp[Y][X], m_comp[Z][X],
                         m_comp[X][Y], m_comp[Y][Y], m_comp[Z][Y],
                         m_comp[X][Z], m_comp[Y][Z], m_comp[Z][Z] ) );
}




// -----------------------------------------------------------------------------
// Operator +=
template <typename T>
__HOSTDEVICE__
Matrix3<T>& Matrix3<T>::operator += ( Matrix3<T> const& m )
{
    setValue(
        m_comp[X][X] + m[X][X], m_comp[X][Y] + m[X][Y], m_comp[X][Z] + m[X][Z],
        m_comp[Y][X] + m[Y][X], m_comp[Y][Y] + m[Y][Y], m_comp[Y][Z] + m[Y][Z],
        m_comp[Z][X] + m[Z][X], m_comp[Z][Y] + m[Z][Y], m_comp[Z][Z] + m[Z][Z]);
    return ( *this );
}




// -----------------------------------------------------------------------------
// Operator -=
template <typename T>
__HOSTDEVICE__
Matrix3<T>& Matrix3<T>::operator -= ( Matrix3<T> const& m )
{
    setValue(
        m_comp[X][X] - m[X][X], m_comp[X][Y] - m[X][Y], m_comp[X][Z] - m[X][Z],
        m_comp[Y][X] - m[Y][X], m_comp[Y][Y] - m[Y][Y], m_comp[Y][Z] - m[Y][Z],
        m_comp[Z][X] - m[Z][X], m_comp[Z][Y] - m[Z][Y], m_comp[Z][Z] - m[Z][Z]);
    return ( *this );
}




// -----------------------------------------------------------------------------
// Operator *= by a scalar
template <typename T>
__HOSTDEVICE__
Matrix3<T>& Matrix3<T>::operator *= ( T d )
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
__HOSTDEVICE__
Matrix3<T>& Matrix3<T>::operator *= ( Matrix3<T> const& m )
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
__HOSTDEVICE__
Vector3<T>& Matrix3<T>::operator [] ( unsigned int i ) const
{
    return ( *( Vector3<T>* )m_comp[i] );
}




// -----------------------------------------------------------------------------
// Assign operator to another matrix
template <typename T>
__HOSTDEVICE__
Matrix3<T>& Matrix3<T>::operator = ( Matrix3<T> const& m )
{
    if ( &m != this )
    {
        for ( int i = 0; i < 3; ++i )
            for ( int j = 0; j < 3; ++j )
                m_comp[i][j] = m.m_comp[i][j];
    }
    return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator -
template <typename T>
__HOSTDEVICE__ 
Matrix3<T>& Matrix3<T>::operator - ()
{
  setValue(
        - m_comp[X][X], - m_comp[X][Y], - m_comp[X][Z],
        - m_comp[Y][X], - m_comp[Y][Y], - m_comp[Y][Z],
        - m_comp[Z][X], - m_comp[Z][Y], - m_comp[Z][Z] );
  return ( *this );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
std::ostream& operator << ( std::ostream& fileOut, 
                            Matrix3<T> const& m )
{
    fileOut << m[X] << std::endl << m[Y] << std::endl << m[Z];
    return ( fileOut );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
std::istream& operator >> ( std::istream& fileIn, 
                            Matrix3<T>& m )
{
    fileIn >> m[X] >> m[Y] >> m[Z];
    return ( fileIn );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Matrix3<float>;
template class Matrix3<double>;

#define X( T ) \
template std::ostream& operator << <T>( std::ostream& fileOut,                 \
                                        Matrix3<T> const& m );                 \
                                                                               \
template std::istream& operator >> <T>( std::istream& fileIn,                  \
                                        Matrix3<T>& m );
X( float )
X( double )
#undef X
