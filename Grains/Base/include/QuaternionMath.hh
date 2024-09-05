#ifndef _QUATERNIONMATH_HH_
#define _QUATERNIONMATH_HH_


#include "Quaternion.hh"
#include "VectorMath.hh"


// =============================================================================
/** @brief Miscellaneous Quaternion functions and operators as header-only.
   
    Defining important quaternion functions and operators as static functions.
    It will increase the binary size, but the performance gain is much more
    appreciated.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name Quaternion math functions and operators */
//@{
/** @brief Returns the norm of the quaternion
@param q the quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE T norm( Quaternion<T> const& q )
{
    return( sqrt( norm2( q.getVector ) + q.getScalar() * q.getScalar() ) );
}




// -----------------------------------------------------------------------------
/** @brief Returns the norm squared of the quaternion
@param q the quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE T norm2( Quaternion<T> const& q )
{
    return( norm2( q.getVector ) + q.getScalar() * q.getScalar() );
}




// -----------------------------------------------------------------------------
/** @brief Returns the conjugate of the quaternion
@param q the quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> conjugate( Quaternion<T> const& q )
{
    return ( Quaternion<T>( -q.getVector(), q.getScalar ) );
}




// -----------------------------------------------------------------------------
/** @brief Returns the inverse of the quaternion
@param q the quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> inverse( Quaternion<T> const& q )
{
    return ( ( T( 1 ) / norm( q ) ) * conjugate( q ) );
}




// -----------------------------------------------------------------------------
/** @brief Sum of 2 quaternions, i.e., q1 + q2
@param q1 1st quaternion 
@param q2 2nd quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> operator + ( Quaternion<T> const& q1,
                                         Quaternion<T> const& q2 )
{
    return ( Quaternion<T>( q1[0] + q2[0], 
                            q1[1] + q2[1],
                            q1[2] + q2[2],
                            q1[3] + q2[3] ) );
}




// ----------------------------------------------------------------------------- 
/** @brief Subtraction of 2 quaternions, i.e., q1 - q2
@param q1 1st quaternion
@param q2 2nd quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> operator - ( Quaternion<T> const& q1,
                                         Quaternion<T> const& q2 )
{
    return ( Quaternion<T>( q1[0] - q2[0], 
                            q1[1] - q2[1],
                            q1[2] - q2[2],
                            q1[3] - q2[3] ) );
}




// ----------------------------------------------------------------------------
/** @brief Multiplication by a scalar
@param d the multiplication factor
@param q the quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> operator * ( T d,
                                         Quaternion<T> const& q )
{
    return ( Quaternion<T>( d * q[0], 
                            d * q[1],
                            d * q[2],
                            d * q[3] ) );
}




// ----------------------------------------------------------------------------
/** @brief double product q1 x q2 of 2 quaternions
@param q1 1st quaternion
@param q2 2nd quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> operator * ( Quaternion<T> const& q1,
                                         Quaternion<T> const& q2 )
{
    T w1 = q1.getScalar();
    Vector3<T> v1 = q1.getVector();
    T w2 = q2.getScalar();
    Vector3<T> v2 = q2.getVector();
    T tmp = ( w1 * w2 ) - ( v1 * v2 );
    Vector3<T> vtmp( ( v1 ^ v2 ) + ( w1 * v2 ) + ( w2 * v1 ) );
    return ( Quaternion<T>( vtmp, tmp ) );
}




// ----------------------------------------------------------------------------
/** @brief double product on the right of a quaternion by a vector [ 0, v ],
i.e., q x [ 0, v ]
@param v the vector
@param q the quaternion */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> operator * ( Quaternion<T> const& q,
                                         Vector3<T> const& v )
{
    T tmp = - q.getVector() * v;
    Vector3<T> vtmp( ( q.getVector() ^ v ) + ( q.getScalar() * v ) );
    return ( Quaternion<T>( vtmp, tmp ) );
}




// ----------------------------------------------------------------------------
/** @brief double product on the left of a quaternion by a vector [ 0, v ], 
i.e., [ 0, v ] x q
@param q the quaternion
@param v the vector */
template <typename T>
__HOSTDEVICE__
static INLINE Quaternion<T> operator * ( Vector3<T> const& v,
                                         Quaternion<T> const& q )
{
    T tmp = -v * q.getVector();
	Vector3<T> vtmp( ( v ^ q.getVector() ) + ( q.getScalar() * v ) );
	return ( Quaternion<T>( vtmp, tmp ) );
}
//@}

#endif