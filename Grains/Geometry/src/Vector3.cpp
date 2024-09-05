#include "VectorMath.hh"
#include "Vector3.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
Vector3<T>::Vector3( T def )
{
    m_comp[X] = m_comp[Y] = m_comp[Z] = def;
}




// -----------------------------------------------------------------------------
// Constructor with 3 components as inputs
template <typename T>
__HOSTDEVICE__
Vector3<T>::Vector3( T x,
                     T y,
                     T z )
{
    m_comp[X] = x;
    m_comp[Y] = y;
    m_comp[Z] = z;
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__HOSTDEVICE__
Vector3<T>::Vector3( Vector3<T> const& vec )
{
    m_comp[X] = vec.m_comp[X];
    m_comp[Y] = vec.m_comp[Y];
    m_comp[Z] = vec.m_comp[Z];
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Vector3<T>::~Vector3()
{}




// -----------------------------------------------------------------------------
/* Sets the components */
template <typename T>
__HOSTDEVICE__
void Vector3<T>::setValue( T x,
                           T y,
                           T z )
{
    m_comp[X] = x;
    m_comp[Y] = y;
    m_comp[Z] = z;
}




// -----------------------------------------------------------------------------
// Unitary nomalization operator
template <typename T>
__HOSTDEVICE__
void Vector3<T>::normalize()
{
    *this /= this->norm();
}




// -----------------------------------------------------------------------------
// Returns a vector corresponding to the normalized vector
template <typename T>
__HOSTDEVICE__
Vector3<T> Vector3<T>::normalized() const
{
    return ( *this / this->norm() );
}




// -----------------------------------------------------------------------------
// Returns the norm of the vector
template <typename T>
__HOSTDEVICE__
T Vector3<T>::norm() const
{
    return ( sqrt( m_comp[X] * m_comp[X] + 
                   m_comp[Y] * m_comp[Y] + 
                   m_comp[Z] * m_comp[Z] ) );
}



// -----------------------------------------------------------------------------
// Returns the norm squared of the vector
template <typename T>
__HOSTDEVICE__
T Vector3<T>::norm2() const
{
    return ( m_comp[X] * m_comp[X] + 
             m_comp[Y] * m_comp[Y] + 
             m_comp[Z] * m_comp[Z] );
}




// -----------------------------------------------------------------------------
// Determines if the vector is approximately zero or not
template <typename T>
__HOSTDEVICE__
bool Vector3<T>::isApproxZero( T tol ) const
{
    return ( fabs( m_comp[X] ) < tol && 
             fabs( m_comp[Y] ) < tol &&
             fabs( m_comp[Z] ) < tol );
    
}




// -----------------------------------------------------------------------------
// Rounds components to +-tol
template <typename T>
__HOSTDEVICE__
void Vector3<T>::round( T tol )
{
    m_comp[X] = fabs( m_comp[X] ) < tol ? T( 0 ) : m_comp[X];
    m_comp[Y] = fabs( m_comp[Y] ) < tol ? T( 0 ) : m_comp[Y];
    m_comp[Z] = fabs( m_comp[Z] ) < tol ? T( 0 ) : m_comp[Z];
}




// -----------------------------------------------------------------------------
// Sets components to zero
template <typename T>
__HOSTDEVICE__
void Vector3<T>::reset()
{
    m_comp[X] = m_comp[Y] = m_comp[Z] = T( 0 );
}




// -----------------------------------------------------------------------------
// Operator +=
template <typename T>
__HOSTDEVICE__
Vector3<T>& Vector3<T>::operator += ( Vector3<T> const& vec )
{
    m_comp[X] += vec.m_comp[X];
    m_comp[Y] += vec.m_comp[Y];
    m_comp[Z] += vec.m_comp[Z];
    return ( *this );
}




// -----------------------------------------------------------------------------
// Operator -=
template <typename T>
__HOSTDEVICE__
Vector3<T>& Vector3<T>::operator -= ( Vector3<T> const& vec )
{
    m_comp[X] -= vec.m_comp[X];
    m_comp[Y] -= vec.m_comp[Y];
    m_comp[Z] -= vec.m_comp[Z];
    return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator *= by a scalar
template <typename T>
__HOSTDEVICE__
Vector3<T>& Vector3<T>::operator *= ( T d )
{
    m_comp[X] *= d;
    m_comp[Y] *= d;
    m_comp[Z] *= d;
    return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator /= by a scalar
template <typename T>
__HOSTDEVICE__
Vector3<T>& Vector3<T>::operator /= ( T d )
{
    m_comp[X] /= d;
    m_comp[Y] /= d;
    m_comp[Z] /= d;
    return ( *this );
}




// -----------------------------------------------------------------------------
// ith component accessor
template <typename T>
__HOSTDEVICE__
T Vector3<T>::operator [] ( size_t i ) const
{
    return ( m_comp[i] );
}




// -----------------------------------------------------------------------------
// ith component accessor - modifiable lvalue
template <typename T>
__HOSTDEVICE__
T& Vector3<T>::operator [] ( size_t i )
{
    return ( m_comp[i] );
}




// -----------------------------------------------------------------------------
// Equal operator to another Vector3 object
template <typename T>
__HOSTDEVICE__
Vector3<T>& Vector3<T>::operator = ( Vector3<T> const& vec )
{
    if ( &vec != this )
    {
        m_comp[X] = vec.m_comp[X];
        m_comp[Y] = vec.m_comp[Y];
        m_comp[Z] = vec.m_comp[Z];
    }
    return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator -. Return an object with negative components
template <typename T>
__HOSTDEVICE__
Vector3<T> Vector3<T>::operator - () const
{
    return ( Vector3<T>( - m_comp[X], - m_comp[Y], - m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Comparison operator
template <typename T>
__HOSTDEVICE__
bool Vector3<T>::operator == ( Vector3<T> const& vec ) const
{
    return ( m_comp[X] == vec[X] && 
             m_comp[Y] == vec[Y] &&
             m_comp[Z] == vec[Z] );
}




// -----------------------------------------------------------------------------
// Difference operator
template <typename T>
__HOSTDEVICE__
bool Vector3<T>::operator != ( Vector3<T> const& vec ) const
{
    return ( m_comp[X] != vec[X] ||
             m_comp[Y] != vec[Y] ||
             m_comp[Z] != vec[Z] );
}




// -----------------------------------------------------------------------------
// Conversion operator to float
template <>
__HOSTDEVICE__
Vector3<double>::operator Vector3<float> () const
{
    return ( Vector3<float> ( (float) m_comp[X],
                              (float) m_comp[Y],
                              (float) m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Output operator
template <typename T>
__HOST__
std::ostream& operator << ( std::ostream& fileOut, 
                            Vector3<T> const& v )
{
    fileOut << v[X] << " " << v[Y] << " " << v[Z];
    return ( fileOut );
}




// -----------------------------------------------------------------------------
// Input operator
template <typename T>
__HOST__
std::istream& operator >> ( std::istream& fileIn, 
                            Vector3<T>& v )
{
    fileIn >> v[X] >> v[Y] >> v[Z];
    return ( fileIn );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Vector3<float>;
template class Vector3<double>;

#define X( T ) \
template std::ostream& operator << <T>( std::ostream& fileOut,                 \
                                        Vector3<T> const& v );                 \
                                                                               \
template std::istream& operator >> <T>( std::istream& fileIn,                  \
                                        Vector3<T>& v );
X( float )
X( double )
#undef X