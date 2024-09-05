#ifndef _MATRIXMATH_HH_
#define _MATRIXMATH_HH_


#include "Vector3.hh"
#include "Matrix3.hh"
#include "VectorMath.hh"


// =============================================================================
/** @brief Miscellaneous Matrix3 functions and operators as header-only.
   
    Defining important matrix functions and operators here as static functions.
    It will increase the binary size, but the performance gain is much more
    appreciated.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name Matrix3 math functions and operators */
//@{
/** @brief Returns the determinant of the matrix
@param m the matrix */
template <typename T>
__HOSTDEVICE__
static INLINE T determinant( Matrix3<T> const& m )
{
    return ( m[X][X] * ( m[Y][Y] * m[Z][Z] - m[Y][Z] * m[Z][Y] ) -
             m[X][Y] * ( m[Y][X] * m[Z][Z] - m[Y][Z] * m[Z][X] ) +
             m[X][Z] * ( m[Y][X] * m[Z][Y] - m[Y][Y] * m[Z][X] ) );
}




// -----------------------------------------------------------------------------
/** @brief Returns the transposed matrix
@param m the matrix */
template <typename T>
__HOSTDEVICE__
static INLINE Matrix3<T> transpose( Matrix3<T> const& m )
{
    return ( Matrix3<T>( m[X][X], m[Y][X], m[Z][X],
                         m[X][Y], m[Y][Y], m[Z][Y],
                         m[X][Z], m[Y][Z], m[Z][Z] ) );
}




// -----------------------------------------------------------------------------
/** @brief Returns the inverse of the matrix
@param m the matrix */
template <typename T>
__HOSTDEVICE__
static INLINE Matrix3<T> inverse( Matrix3<T> const& m )
{
    Vector3<T> co( m[Y][Y] * m[Z][Z] - m[Y][Z] * m[Z][Y],
                   m[Y][Z] * m[Z][X] - m[Y][X] * m[Z][Z],
                   m[Y][X] * m[Z][Y] - m[Y][Y] * m[Z][X] );
    T d = m[X] * co;
    if ( fabs( d ) < HIGHEPS<T> )
        printf( "Matrix is not inversible!\n" );
    T s = T( 1 ) / d;
    return ( Matrix3<T>( co[X] * s,
                         ( m[X][Z] * m[Z][Y] - m[X][Y] * m[Z][Z] ) * s,
                         ( m[X][Y] * m[Y][Z] - m[X][Z] * m[Y][Y] ) * s,
                         co[Y] * s,
                         ( m[X][X] * m[Z][Z] - m[X][Z] * m[Z][X] ) * s,
                         ( m[X][Z] * m[Y][X] - m[X][X] * m[Y][Z] ) * s,
                         co[Z] * s,
                         ( m[X][Y] * m[Z][X] - m[X][X] * m[Z][Y] ) * s,
                         ( m[X][X] * m[Y][Y] - m[X][Y] * m[Y][X] ) * s ) );
}




// -----------------------------------------------------------------------------
/** @brief Matrces addition
@param m1 first matrix
@param m2 second matrix */
template <typename T>
__HOSTDEVICE__
static INLINE Matrix3<T> operator + ( Matrix3<T> const& m1,
                                      Matrix3<T> const& m2 )
{
    return ( Matrix3<T>( 
            m1[X][X] + m2[X][X], m1[X][Y] + m2[X][Y], m1[X][Z] + m2[X][Z],
            m1[Y][X] + m2[Y][X], m1[Y][Y] + m2[Y][Y], m1[Y][Z] + m2[Y][Z],
            m1[Z][X] + m2[Z][X], m1[Z][Y] + m2[Z][Y], m1[Z][Z] + m2[Z][Z] ) );
}                                      




// -----------------------------------------------------------------------------
/** @brief Scalar-matrix product
@param c the scalar
@param m the matrix */
template <typename T>
__HOSTDEVICE__
static INLINE Matrix3<T> operator * ( T c,
                                      Matrix3<T> const& m )
{
    return ( Matrix3<T>( c * m[X][X], c * m[X][Y], c * m[X][Z],
                         c * m[Y][X], c * m[Y][Y], c * m[Y][Z],
                         c * m[Z][X], c * m[Z][Y], c * m[Z][Z] ) );
}                                      




// -----------------------------------------------------------------------------
/** @brief Matrix-vector product
@param m the matrix
@param v the vector */
template <typename T>
__HOSTDEVICE__ 
static INLINE Vector3<T> operator * ( Matrix3<T> const& m,
                                      Vector3<T> const& v )
{
    return ( Vector3<T>( m[X][X] * v[X] + m[X][Y] * v[Y] + m[X][X] * v[Z],
                         m[Y][X] * v[X] + m[Y][Y] * v[Y] + m[Y][X] * v[Z],
                         m[Z][X] * v[X] + m[Z][Y] * v[Y] + m[Z][X] * v[Z] ) );
}




// -----------------------------------------------------------------------------
/** @brief Vector-matrix product
@param v the vector
@param m the matrix */
template <typename T>
__HOSTDEVICE__
static INLINE Vector3<T> operator * ( Vector3<T> const& v,
                                      Matrix3<T> const& m )
{
    Matrix3<T> tr( transpose( m ) );
    return ( Vector3<T>( tr[X] * v, tr[Y] * v, tr[Z] * v ) );
}




// -----------------------------------------------------------------------------
/** @brief Matrix-matrix product
@param m right matrix */
template <typename T>
__HOSTDEVICE__
static INLINE Matrix3<T> operator * ( Matrix3<T> const& m1,
                                      Matrix3<T> const& m2 )
{
    return ( Matrix3<T>(
            m1[X][X] * m2[X][X] + m1[X][Y] * m2[Y][X] + m1[X][Z] * m2[Z][X],
            m1[X][X] * m2[X][Y] + m1[X][Y] * m2[Y][Y] + m1[X][Z] * m2[Z][Y],
            m1[X][X] * m2[X][Z] + m1[X][Y] * m2[Y][Z] + m1[X][Z] * m2[Z][Z],
            m1[Y][X] * m2[X][X] + m1[Y][Y] * m2[Y][X] + m1[Y][Z] * m2[Z][X],
            m1[Y][X] * m2[X][Y] + m1[Y][Y] * m2[Y][Y] + m1[Y][Z] * m2[Z][Y],
            m1[Y][X] * m2[X][Z] + m1[Y][Y] * m2[Y][Z] + m1[Y][Z] * m2[Z][Z],
            m1[Z][X] * m2[X][X] + m1[Z][Y] * m2[Y][X] + m1[Z][Z] * m2[Z][X],
            m1[Z][X] * m2[X][Y] + m1[Z][Y] * m2[Y][Y] + m1[Z][Z] * m2[Z][Y],
            m1[Z][X] * m2[X][Z] + m1[Z][Y] * m2[Y][Z] + m1[Z][Z] * m2[Z][Z] ) );
}




// -----------------------------------------------------------------------------
// /** @brief Returns a matrix that rotates vector src to vector dest,
// i.e. dest = mat * src
// @param src the source vector
// @param dest the destination vector */
// Matrix3<T> getRotationMatrix( Vector3 const& src, Vector3 const& dest );
//@}


#endif