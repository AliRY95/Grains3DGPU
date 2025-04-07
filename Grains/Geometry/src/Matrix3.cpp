#include "Matrix3.hh"
#include "VectorMath.hh"

// -------------------------------------------------------------------------------------------------
// Default constructor. Matrix is initialized to the identity matrix
template <typename T>
__HOSTDEVICE__ Matrix3<T>::Matrix3()
{
    setValue(T(1), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(1));
}

// -------------------------------------------------------------------------------------------------
// Constructor with a 1D array of values as input
template <typename T>
__HOSTDEVICE__ Matrix3<T>::Matrix3(T const* buffer)
{
    setValue(buffer);
}

// -------------------------------------------------------------------------------------------------
// Constructor with 9 components as inputs
template <typename T>
__HOSTDEVICE__ Matrix3<T>::Matrix3(T xx, T xy, T xz, T yx, T yy, T yz, T zx, T zy, T zz)
{
    setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// -------------------------------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__ Matrix3<T>::Matrix3(Matrix3<T> const& mat)
{
    setValue(mat.getBuffer());
}

// -------------------------------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__ Matrix3<T>::~Matrix3()
{
}

// -------------------------------------------------------------------------------------------------
/* Gets the pointer to the buffer */
template <typename T>
__HOSTDEVICE__ T const* Matrix3<T>::getBuffer() const
{
    return (m_comp);
}

// -------------------------------------------------------------------------------------------------
// Sets the matrix to a 1D array of 9 values as input
template <typename T>
__HOSTDEVICE__ void Matrix3<T>::setValue(T const* buffer)
{
    m_comp[XX] = buffer[XX];
    m_comp[XY] = buffer[XY];
    m_comp[XZ] = buffer[XZ];
    m_comp[YX] = buffer[YX];
    m_comp[YY] = buffer[YY];
    m_comp[YZ] = buffer[YZ];
    m_comp[ZX] = buffer[ZX];
    m_comp[ZY] = buffer[ZY];
    m_comp[ZZ] = buffer[ZZ];
}

// -------------------------------------------------------------------------------------------------
// Sets the matrix with all 9 components as inputs
template <typename T>
__HOSTDEVICE__ void Matrix3<T>::setValue(T xx, T xy, T xz, T yx, T yy, T yz, T zx, T zy, T zz)
{
    m_comp[XX] = xx;
    m_comp[XY] = xy;
    m_comp[XZ] = xz;
    m_comp[YX] = yx;
    m_comp[YY] = yy;
    m_comp[YZ] = yz;
    m_comp[ZX] = zx;
    m_comp[ZY] = zy;
    m_comp[ZZ] = zz;
}

// -------------------------------------------------------------------------------------------------
// Returns a matrix with positive components
template <typename T>
__HOSTDEVICE__ Matrix3<T> Matrix3<T>::absolute() const
{
    return (Matrix3<T>(fabs(m_comp[XX]),
                       fabs(m_comp[XY]),
                       fabs(m_comp[XZ]),
                       fabs(m_comp[YX]),
                       fabs(m_comp[YY]),
                       fabs(m_comp[YZ]),
                       fabs(m_comp[ZX]),
                       fabs(m_comp[ZY]),
                       fabs(m_comp[ZZ])));
}

// -------------------------------------------------------------------------------------------------
// Returns the determinant of the matrix
template <typename T>
__HOSTDEVICE__ T Matrix3<T>::determinant() const
{
    return (m_comp[XX] * (m_comp[YY] * m_comp[ZZ] - m_comp[YZ] * m_comp[ZY])
            + m_comp[XY] * (m_comp[YZ] * m_comp[ZX] - m_comp[YX] * m_comp[ZZ])
            + m_comp[XZ] * (m_comp[YX] * m_comp[ZY] - m_comp[YY] * m_comp[ZX]));
}

// -------------------------------------------------------------------------------------------------
// Returns the inverse of the matrix
template <typename T>
__HOSTDEVICE__ Matrix3<T> Matrix3<T>::inverse() const
{
    T __RESTRICT__ out[9];
    out[XX] = (m_comp[YY] * m_comp[ZZ] - m_comp[YZ] * m_comp[ZY]);
    out[YX] = (m_comp[YZ] * m_comp[ZX] - m_comp[YX] * m_comp[ZZ]);
    out[ZX] = (m_comp[YX] * m_comp[ZY] - m_comp[YY] * m_comp[ZX]);
    T det   = m_comp[XX] * out[XX] + m_comp[XY] * out[YX] + m_comp[XZ] * out[ZX];
    if(fabs(det) < HIGHEPS<T>)
        printf("Matrix is not inversible!\n");
    T s     = T(1) / det;
    out[ZZ] = s * (out[XX]);
    out[XY] = s * (m_comp[XZ] * m_comp[ZY] - m_comp[XY] * m_comp[ZZ]);
    out[XZ] = s * (m_comp[XY] * m_comp[YZ] - m_comp[XZ] * m_comp[YY]);
    out[YX] = s * (out[XZ]);
    out[YY] = s * (m_comp[XX] * m_comp[ZZ] - m_comp[XZ] * m_comp[ZX]);
    out[YZ] = s * (m_comp[XZ] * m_comp[YY] - m_comp[XX] * m_comp[YZ]);
    out[ZX] = s * (out[ZX]);
    out[ZY] = s * (m_comp[XY] * m_comp[ZX] - m_comp[XX] * m_comp[ZY]);
    out[ZZ] = s * (m_comp[XX] * m_comp[YY] - m_comp[XY] * m_comp[YX]);
    return (Matrix3<T>(out));
}

// -------------------------------------------------------------------------------------------------
// Returns the transposed matrix
template <typename T>
__HOSTDEVICE__ Matrix3<T> Matrix3<T>::transpose() const
{
    return (Matrix3<T>(m_comp[XX],
                       m_comp[YX],
                       m_comp[ZX],
                       m_comp[XY],
                       m_comp[YY],
                       m_comp[ZY],
                       m_comp[XZ],
                       m_comp[YZ],
                       m_comp[ZZ]));
}

// -------------------------------------------------------------------------------------------------
// Operator +=
template <typename T>
__HOSTDEVICE__ Matrix3<T>& Matrix3<T>::operator+=(Matrix3<T> const& m)
{
    T const* b = m.getBuffer();
    setValue(m_comp[XX] + b[XX],
             m_comp[XY] + b[XY],
             m_comp[XZ] + b[XZ],
             m_comp[YX] + b[YX],
             m_comp[YY] + b[YY],
             m_comp[YZ] + b[YZ],
             m_comp[ZX] + b[ZX],
             m_comp[ZY] + b[ZY],
             m_comp[ZZ] + b[ZZ]);
    return (*this);
}

// -------------------------------------------------------------------------------------------------
// Operator -=
template <typename T>
__HOSTDEVICE__ Matrix3<T>& Matrix3<T>::operator-=(Matrix3<T> const& m)
{
    T const* b = m.getBuffer();
    setValue(m_comp[XX] - b[XX],
             m_comp[XY] - b[XY],
             m_comp[XZ] - b[XZ],
             m_comp[YX] - b[YX],
             m_comp[YY] - b[YY],
             m_comp[YZ] - b[YZ],
             m_comp[ZX] - b[ZX],
             m_comp[ZY] - b[ZY],
             m_comp[ZZ] - b[ZZ]);
    return (*this);
}

// -------------------------------------------------------------------------------------------------
// Operator *= by a scalar
template <typename T>
__HOSTDEVICE__ Matrix3<T>& Matrix3<T>::operator*=(T d)
{
    setValue(d * m_comp[XX],
             d * m_comp[XY],
             d * m_comp[XZ],
             d * m_comp[YX],
             d * m_comp[YY],
             d * m_comp[YZ],
             d * m_comp[ZX],
             d * m_comp[ZY],
             d * m_comp[ZZ]);
    return (*this);
}

// -------------------------------------------------------------------------------------------------
// Operator *= by a matrix
template <typename T>
__HOSTDEVICE__ Matrix3<T>& Matrix3<T>::operator*=(Matrix3<T> const& m)
{
    T const* b = m.getBuffer();
    setValue(m_comp[XX] * b[XX] + m_comp[XY] * b[YX] + m_comp[XZ] * b[ZX],
             m_comp[XX] * b[XY] + m_comp[XY] * b[YY] + m_comp[XZ] * b[ZY],
             m_comp[XX] * b[XZ] + m_comp[XY] * b[YZ] + m_comp[XZ] * b[ZZ],
             m_comp[YX] * b[XX] + m_comp[YY] * b[YX] + m_comp[YZ] * b[ZX],
             m_comp[YX] * b[XY] + m_comp[YY] * b[YY] + m_comp[YZ] * b[ZY],
             m_comp[YX] * b[XZ] + m_comp[YY] * b[YZ] + m_comp[YZ] * b[ZZ],
             m_comp[ZX] * b[XX] + m_comp[ZY] * b[YX] + m_comp[ZZ] * b[ZX],
             m_comp[ZX] * b[XY] + m_comp[ZY] * b[YY] + m_comp[ZZ] * b[ZY],
             m_comp[ZX] * b[XZ] + m_comp[ZY] * b[YZ] + m_comp[ZZ] * b[ZZ]);
    return (*this);
}

// -------------------------------------------------------------------------------------------------
// i-th row accessor
template <typename T>
__HOSTDEVICE__ Vector3<T>& Matrix3<T>::operator[](unsigned int i) const
{
    return (*(Vector3<T>*)(m_comp + 3 * i));
}

// -------------------------------------------------------------------------------------------------
// Assign operator to another matrix
template <typename T>
__HOSTDEVICE__ Matrix3<T>& Matrix3<T>::operator=(Matrix3<T> const& m)
{
    if(&m != this)
        setValue(m.getBuffer());
    return (*this);
}

// -------------------------------------------------------------------------------------------------
// Unitary operator -
template <typename T>
__HOSTDEVICE__ Matrix3<T>& Matrix3<T>::operator-()
{
    setValue(-m_comp[XX],
             -m_comp[XY],
             -m_comp[XZ],
             -m_comp[YX],
             -m_comp[YY],
             -m_comp[YZ],
             -m_comp[ZX],
             -m_comp[ZY],
             -m_comp[ZZ]);
    return (*this);
}

// -------------------------------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__ std::ostream& operator<<(std::ostream& fileOut, Matrix3<T> const& m)
{
    fileOut << m[X] << std::endl << m[Y] << std::endl << m[Z];
    return (fileOut);
}

// -------------------------------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__ std::istream& operator>>(std::istream& fileIn, Matrix3<T>& m)
{
    fileIn >> m[X] >> m[Y] >> m[Z];
    return (fileIn);
}

// -------------------------------------------------------------------------------------------------
// Explicit instantiation
template class Matrix3<float>;
template class Matrix3<double>;

#define X(T)                                                                            \
    template std::ostream& operator<< <T>(std::ostream & fileOut, Matrix3<T> const& m); \
                                                                                        \
    template std::istream& operator>> <T>(std::istream & fileIn, Matrix3<T> & m);
X(float)
X(double)
#undef X
