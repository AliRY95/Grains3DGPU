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
__HOSTDEVICE__ static INLINE T norm(Vector3<T> const& v) noexcept
{
    T const* __RESTRICT__ buffer = v.getBuffer();
    return (sqrt(buffer[0] * buffer[0] + buffer[1] * buffer[1]
                 + buffer[2] * buffer[2]));
}

// -----------------------------------------------------------------------------
/** @brief Returns the norm squared of the vector
@param v the vector */
template <typename T>
__HOSTDEVICE__ static INLINE T norm2(Vector3<T> const& v) noexcept
{
    T const* __RESTRICT__ buffer = v.getBuffer();
    return (buffer[0] * buffer[0] + buffer[1] * buffer[1]
            + buffer[2] * buffer[2]);
}

// -----------------------------------------------------------------------------
/** @brief Determines if the vector is approximately zero or not
@param v the vector */
template <typename T>
__HOSTDEVICE__ static INLINE bool isApproxZero(Vector3<T> const& v,
                                               T tol = HIGHEPS<T>) noexcept
{
    T const* __RESTRICT__ buffer = v.getBuffer();
    return (fabs(buffer[0]) < tol && fabs(buffer[1]) < tol
            && fabs(buffer[2]) < tol);
}

// -----------------------------------------------------------------------------
/** @brief Vectors addition
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator+(Vector3<T> const& v1,
                                                  Vector3<T> const& v2) noexcept
{
    T const* __RESTRICT__ b1 = v1.getBuffer();
    T const* __RESTRICT__ b2 = v2.getBuffer();
    T __RESTRICT__        out[3];
    for(unsigned int i = 0; i < 3; ++i)
        out[i] = b1[i] + b2[i];
    return (Vector3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Vectors subtraction
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator-(Vector3<T> const& v1,
                                                  Vector3<T> const& v2) noexcept
{
    T const* __RESTRICT__ b1 = v1.getBuffer();
    T const* __RESTRICT__ b2 = v2.getBuffer();
    T __RESTRICT__        out[3];
    for(unsigned int i = 0; i < 3; ++i)
        out[i] = b1[i] - b2[i];
    return (Vector3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Multiplication by a scalar
@param d the multiplication factor
@param v the vector */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator*(T                 d,
                                                  Vector3<T> const& v) noexcept
{
    T const* __RESTRICT__ buffer = v.getBuffer();
    T __RESTRICT__        out[3];
    for(unsigned int i = 0; i < 3; ++i)
        out[i] = d * buffer[i];
    return (Vector3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Division by a scalar
@param d division factor
@param v the vector */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator/(Vector3<T> const& v,
                                                  T                 d) noexcept
{
    T const* __RESTRICT__ buffer = v.getBuffer();
    T __RESTRICT__        out[3];
    for(unsigned int i = 0; i < 3; ++i)
        out[i] = buffer[i] / d;
    return (Vector3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Dot product
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__ static INLINE T operator*(Vector3<T> const& v1,
                                         Vector3<T> const& v2) noexcept
{
    T const* __RESTRICT__ b1  = v1.getBuffer();
    T const* __RESTRICT__ b2  = v2.getBuffer();
    T                     out = T(0);
    for(unsigned int i = 0; i < 3; ++i)
        out += b1[i] * b2[i];
    return (out);
}

// -----------------------------------------------------------------------------
/** @brief Cross product v1 x v2
@param v1 1st vector
@param v2 2nd vector */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator^(Vector3<T> const& v1,
                                                  Vector3<T> const& v2) noexcept
{
    T const* __RESTRICT__ b1 = v1.getBuffer();
    T const* __RESTRICT__ b2 = v2.getBuffer();
    T __RESTRICT__        out[3];
    out[0] = b1[1] * b2[2] - b1[2] * b2[1];
    out[1] = b1[2] * b2[0] - b1[0] * b2[2];
    out[2] = b1[0] * b2[1] - b1[1] * b2[0];
    return (Vector3<T>(out));
}
//@}

#endif