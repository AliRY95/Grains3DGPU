#ifndef _VECTORMATH_HH_
#define _VECTORMATH_HH_


#include "Vector3.hh"


// =============================================================================
/** @brief Miscellaneous Vector3 functions and operators as header-only.
   
    Defining important vector functions and operators here as static functions.
    It will increase the binary size, but the performance gain is much more
    appreciated.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name Vector3 math functions and operators */
//@{
/** @brief Returns the norm of the vector
@param v the vector */
template <typename T>
__HOSTDEVICE__
static INLINE T norm( Vector3<T> const& v )
{
    return ( sqrt( v[X] * v[X] + 
                   v[Y] * v[Y] + 
                   v[Z] * v[Z] ) );
}




// -----------------------------------------------------------------------------
/** @brief Returns the norm squared of the vector
@param v the vector */
template <typename T>
__HOSTDEVICE__
static INLINE T norm2( Vector3<T> const& v )
{
    return ( v[X] * v[X] + 
             v[Y] * v[Y] + 
             v[Z] * v[Z] );
}




// -----------------------------------------------------------------------------
/** @brief Determines if the vector is approximately zero or not
@param v the vector */
template <typename T>
__HOSTDEVICE__
static INLINE bool isApproxZero( Vector3<T> const& v, T tol = HIGHEPS<T> )
{
    return ( fabs( v[X] ) < tol && 
             fabs( v[Y] ) < tol &&
             fabs( v[Z] ) < tol );
}




// -----------------------------------------------------------------------------
/** @brief Vectors addition
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__
static INLINE Vector3<T> operator + ( Vector3<T> const& v1,
                                      Vector3<T> const& v2 )
{
    return ( Vector3<T>( v1[X] + v2[X],
                         v1[Y] + v2[Y], 
                         v1[Z] + v2[Z] ) );
}




// -----------------------------------------------------------------------------
/** @brief Vectors subtraction
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__
static INLINE Vector3<T> operator - ( Vector3<T> const& v1,
                                      Vector3<T> const& v2 )
{
    return ( Vector3<T>( v1[X] - v2[X],
                         v1[Y] - v2[Y], 
                         v1[Z] - v2[Z] ) );
}




// -----------------------------------------------------------------------------
/** @brief Multiplication by a scalar
@param d the multiplication factor
@param v the vector */
template <typename T>
__HOSTDEVICE__
static INLINE Vector3<T> operator * ( T d,
                                      Vector3<T> const& v )
{
    return ( Vector3<T>( v[X] * d, v[Y] * d, v[Z] * d ) );
}




// -----------------------------------------------------------------------------
/** @brief Division by a scalar
@param d division factor
@param v the vector */
template <typename T>
__HOSTDEVICE__
static INLINE Vector3<T> operator / ( Vector3<T> const& v,
                                      T d )
{
    return ( Vector3<T>( v[X] / d, v[Y] / d, v[Z] / d ) );
}




// -----------------------------------------------------------------------------
/** @brief Dot product
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__
static INLINE T operator * ( Vector3<T> const& v1,
                             Vector3<T> const& v2 )
{
    return ( v1[X] * v2[X] + 
             v1[Y] * v2[Y] +
             v1[Z] * v2[Z] );
}




// -----------------------------------------------------------------------------
/** @brief Cross product v1 x v2
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__
static INLINE Vector3<T> operator ^ ( Vector3<T> const& v1,
                                      Vector3<T> const& v2 )
{
    return ( Vector3<T>( v1[Y] * v2[Z] - v1[Z] * v2[Y],
                       - v1[X] * v2[Z] + v1[Z] * v2[X],
                         v1[X] * v2[Y] - v1[Y] * v2[X] ) );
}
//@}

#endif