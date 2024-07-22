#ifndef _MISCMATH_HH_
#define _MISCMATH_HH_


#include "Basic.hh"


// =============================================================================
/** @brief Miscellaneous math functions as header-only.

    @author A.Yazdani - 2024 - Construction */
// =============================================================================
/** @name Math functions */
//@{
/** @brief Returns whether a double number is approximately 0 with respect to
a given tolerance
@param x the number
@param tol the tolerance */
__HOSTDEVICE__
static INLINE bool eqz( double x, double tol )
{ 
    return ( fabs(x) <= tol );
}




// -----------------------------------------------------------------------------
/** @brief Returns whether a float number is approximately 0 with respect to
a given tolerance
@param x the number
@param tol the tolerance */
__HOSTDEVICE__
static INLINE bool eqz( float x, float tol )
{ 
    return ( fabsf(x) <= tol );
}




// -----------------------------------------------------------------------------
/** @brief Sets the minimum of 2 real numbers defined as double to these 2
numbers
@param x 1st real number 
@param y 2nd real number */
template <typename T>
__HOSTDEVICE__
static INLINE void set_min( T& x, 
                            T y )
{ 
    if ( x > y ) 
        x = y;
}




// -----------------------------------------------------------------------------
/** @brief Sests the maximum of 2 real numbers defined as double to these 2
numbers
@param x 1st real number 
@param y 2nd real number */ 
template <typename T>
__HOSTDEVICE__
static INLINE void set_max( T& x, 
                            T y )
{ 
    if ( x < y ) 
        x = y; 
}




// -----------------------------------------------------------------------------
/** @brief Returns an angle in radians given an angle in degrees
@param x angle in degrees */
template <typename T>
__HOSTDEVICE__
static INLINE T rads( T x )
{ 
    return ( x * RADS_PER_DEG ); 
}




// -----------------------------------------------------------------------------
/** @brief Returns an angle in degrees given an angle in radians
@param x angle in radians */
template <typename T>
__HOSTDEVICE__
static INLINE T degs( T x ) 
{ 
    return ( x * DEGS_PER_RAD );
}




// -----------------------------------------------------------------------------
/** @brief Returns the sign of the real number
@param x the real number */
template <typename T>
__HOSTDEVICE__
static INLINE int sgn( T x )
{
    return ( ( T( 0 ) < x ) - ( x < T( 0 ) ) );
}




// -----------------------------------------------------------------------------
// /** @brief Returns a matrix that rotates vector src to vector dest,
// i.e. dest = mat * src
// @param src the source vector
// @param dest the destination vector */
// Matrix3<T> getRotationMatrix( Vector3 const& src, Vector3 const& dest );
//@}


#endif