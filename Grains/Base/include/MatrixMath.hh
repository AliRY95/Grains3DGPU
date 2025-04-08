#ifndef _MATRIXMATH_HH_
#define _MATRIXMATH_HH_

#include "Matrix3.hh"
#include "Vector3.hh"
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
__HOSTDEVICE__ static INLINE T determinant(Matrix3<T> const& m) noexcept
{
    T const* __RESTRICT__ buffer = m.getBuffer();
    T out0 = buffer[XX] * (buffer[YY] * buffer[ZZ] - buffer[YZ] * buffer[ZY]);
    T out1 = buffer[XY] * (buffer[YZ] * buffer[ZX] - buffer[YX] * buffer[ZZ]);
    T out2 = buffer[XZ] * (buffer[YX] * buffer[ZY] - buffer[YY] * buffer[ZX]);
    return (out0 + out1 + out2);
}

// -----------------------------------------------------------------------------
/** @brief Returns the transposed matrix
@param m the matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Matrix3<T> transpose(Matrix3<T> const& m) noexcept
{
    T const* __RESTRICT__ buffer = m.getBuffer();
    return (Matrix3<T>(buffer[XX],
                       buffer[YX],
                       buffer[ZX],
                       buffer[XY],
                       buffer[YY],
                       buffer[ZY],
                       buffer[XZ],
                       buffer[YZ],
                       buffer[ZZ]));
}

// -----------------------------------------------------------------------------
/** @brief Returns the inverse of the matrix
@param m the matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Matrix3<T> inverse(Matrix3<T> const& m) noexcept
{
    T const* __RESTRICT__ buffer = m.getBuffer();
    T __RESTRICT__        out[9];
    out[XX] = (buffer[YY] * buffer[ZZ] - buffer[YZ] * buffer[ZY]);
    out[YX] = (buffer[YZ] * buffer[ZX] - buffer[YX] * buffer[ZZ]);
    out[ZX] = (buffer[YX] * buffer[ZY] - buffer[YY] * buffer[ZX]);
    T det = buffer[XX] * out[XX] + buffer[XY] * out[YX] + buffer[XZ] * out[ZX];
    if(fabs(det) < HIGHEPS<T>)
        printf("Matrix is not inversible!\n");
    T s     = T(1) / det;
    out[ZZ] = s * (out[XX]);
    out[XY] = s * (buffer[XZ] * buffer[ZY] - buffer[XY] * buffer[ZZ]);
    out[XZ] = s * (buffer[XY] * buffer[YZ] - buffer[XZ] * buffer[YY]);
    out[YX] = s * (out[XZ]);
    out[YY] = s * (buffer[XX] * buffer[ZZ] - buffer[XZ] * buffer[ZX]);
    out[YZ] = s * (buffer[XZ] * buffer[YY] - buffer[XX] * buffer[YZ]);
    out[ZX] = s * (out[ZX]);
    out[ZY] = s * (buffer[XY] * buffer[ZX] - buffer[XX] * buffer[ZY]);
    out[ZZ] = s * (buffer[XX] * buffer[YY] - buffer[XY] * buffer[YX]);
    return (Matrix3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Matrices addition
@param m1 first matrix
@param m2 second matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Matrix3<T> operator+(Matrix3<T> const& m1,
                                                  Matrix3<T> const& m2) noexcept
{
    T const* __RESTRICT__ b1 = m1.getBuffer();
    T const* __RESTRICT__ b2 = m2.getBuffer();
    T __RESTRICT__        out[9];
    for(unsigned int i = 0; i < 9; ++i)
        out[i] = b1[i] + b2[i];
    return (Matrix3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Matrices subtraction
@param m1 first matrix
@param m2 second matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Matrix3<T> operator-(Matrix3<T> const& m1,
                                                  Matrix3<T> const& m2) noexcept
{
    T const* __RESTRICT__ b1 = m1.getBuffer();
    T const* __RESTRICT__ b2 = m2.getBuffer();
    T __RESTRICT__        out[9];
    for(unsigned int i = 0; i < 9; ++i)
        out[i] = b1[i] - b2[i];
    return (Matrix3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Scalar-matrix product
@param c the scalar
@param m the matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Matrix3<T> operator*(T                 c,
                                                  Matrix3<T> const& m) noexcept
{
    T const* __RESTRICT__ buffer = m.getBuffer();
    T __RESTRICT__        out[9];
    for(unsigned int i = 0; i < 9; ++i)
        out[i] = c * buffer[i];
    return (Matrix3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Matrix-vector product
@param m the matrix
@param v the vector */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator*(Matrix3<T> const& m,
                                                  Vector3<T> const& v) noexcept
{
    T const* __RESTRICT__ bufferM = m.getBuffer();
    T const* __RESTRICT__ bufferV = v.getBuffer();
    T __RESTRICT__        out[3];
    for(unsigned int i = 0; i < 3; ++i)
        out[i] = bufferM[3 * i] * bufferV[0] + bufferM[3 * i + 1] * bufferV[1]
                 + bufferM[3 * i + 2] * bufferV[2];
    return (Vector3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Vector-matrix product
@param v the vector
@param m the matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Vector3<T> operator*(Vector3<T> const& v,
                                                  Matrix3<T> const& m) noexcept
{
    T const* __RESTRICT__ bufferV = v.getBuffer();
    T const* __RESTRICT__ bufferM = m.getBuffer();
    T __RESTRICT__        out[3];
    for(unsigned int i = 0; i < 3; ++i)
        out[i] = bufferM[i] * bufferV[0] + bufferM[i + 3] * bufferV[1]
                 + bufferM[i + 6] * bufferV[2];
    return (Vector3<T>(out));
}

// -----------------------------------------------------------------------------
/** @brief Matrix-matrix product
@param m right matrix */
template <typename T>
__HOSTDEVICE__ static INLINE Matrix3<T> operator*(Matrix3<T> const& m1,
                                                  Matrix3<T> const& m2) noexcept
{
    T const* __RESTRICT__ b1 = m1.getBuffer();
    T const* __RESTRICT__ b2 = m2.getBuffer();
    return (Matrix3<T>(b1[XX] * b2[XX] + b1[XY] * b2[YX] + b1[XZ] * b2[ZX],
                       b1[XX] * b2[XY] + b1[XY] * b2[YY] + b1[XZ] * b2[ZY],
                       b1[XX] * b2[XZ] + b1[XY] * b2[YZ] + b1[XZ] * b2[ZZ],
                       b1[YX] * b2[XX] + b1[YY] * b2[YX] + b1[YZ] * b2[ZX],
                       b1[YX] * b2[XY] + b1[YY] * b2[YY] + b1[YZ] * b2[ZY],
                       b1[YX] * b2[XZ] + b1[YY] * b2[YZ] + b1[YZ] * b2[ZZ],
                       b1[ZX] * b2[XX] + b1[ZY] * b2[YX] + b1[ZZ] * b2[ZX],
                       b1[ZX] * b2[XY] + b1[ZY] * b2[YY] + b1[ZZ] * b2[ZY],
                       b1[ZX] * b2[XZ] + b1[ZY] * b2[YZ] + b1[ZZ] * b2[ZZ]));
}
//@}

#endif