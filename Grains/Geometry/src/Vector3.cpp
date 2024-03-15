#include "Basic.hh"
#include "Vector3.hh"


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__host__ __device__
Vector3<T>::Vector3( T def )
{
    m_comp[X] = m_comp[Y] = m_comp[Z] = def;
}




// -----------------------------------------------------------------------------
// Constructor with 3 components as inputs
template <typename T>
__host__ __device__
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
__host__ __device__
Vector3<T>::Vector3( Vector3<T> const& vec )
{
    m_comp[X] = vec.m_comp[X];
    m_comp[Y] = vec.m_comp[Y];
    m_comp[Z] = vec.m_comp[Z];
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__
Vector3<T>::~Vector3()
{}




// -----------------------------------------------------------------------------
/* Sets the components */
template <typename T>
__host__ __device__
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
__host__ __device__
void Vector3<T>::normalize()
{
    *this /= this->norm();
}




// -----------------------------------------------------------------------------
// Returns a vector corresponding to the normalized vector
template <typename T>
__host__ __device__
Vector3<T> Vector3<T>::normalized() const
{
    return ( *this / this->norm() );
}




// -----------------------------------------------------------------------------
//Returns the norm of the vector
template <typename T>
__host__ __device__
T Vector3<T>::norm() const
{
    return ( sqrt( m_comp[X] * m_comp[X] + 
                   m_comp[Y] * m_comp[Y] + 
                   m_comp[Z] * m_comp[Z] ) );
}



// -----------------------------------------------------------------------------
//Returns the norm of the vector - specialized for floats
template <>
__host__ __device__
float Vector3<float>::norm() const
{
    return ( sqrtf( m_comp[X] * m_comp[X] + 
                    m_comp[Y] * m_comp[Y] + 
                    m_comp[Z] * m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Returns the squared norm of the vector
template <typename T>
__host__ __device__
T Vector3<T>::norm2() const
{
    return ( m_comp[X] * m_comp[X] + 
             m_comp[Y] * m_comp[Y] + 
             m_comp[Z] * m_comp[Z] );
}




// -----------------------------------------------------------------------------
// Determines if the vector is approximately zero or not
template <typename T>
__host__ __device__
bool Vector3<T>::isApproxZero() const
{
    return ( fabs( m_comp[X] ) < EPSILON2 && 
             fabs( m_comp[Y] ) < EPSILON2 &&
             fabs( m_comp[Z] ) < EPSILON2 );
}




// -----------------------------------------------------------------------------
// Determines if the vector is approximately zero - specialized for floats
template <>
__host__ __device__
bool Vector3<float>::isApproxZero() const
{
    return ( fabsf( m_comp[X] ) < EPSILON2 && 
             fabsf( m_comp[Y] ) < EPSILON2 &&
             fabsf( m_comp[Z] ) < EPSILON2 );
}




// -----------------------------------------------------------------------------
// Rounds components to +-tol
template <typename T>
__host__ __device__
void Vector3<T>::round( T tol )
{
    m_comp[X] = fabs( m_comp[X] ) < tol ? 0. : m_comp[X];
    m_comp[Y] = fabs( m_comp[Y] ) < tol ? 0. : m_comp[Y];
    m_comp[Z] = fabs( m_comp[Z] ) < tol ? 0. : m_comp[Z];
}




// -----------------------------------------------------------------------------
// Rounds components to +-tol - specialized for floats
template <>
__host__ __device__
void Vector3<float>::round( float tol )
{
    m_comp[X] = fabsf( m_comp[X] ) < tol ? 0. : m_comp[X];
    m_comp[Y] = fabsf( m_comp[Y] ) < tol ? 0. : m_comp[Y];
    m_comp[Z] = fabsf( m_comp[Z] ) < tol ? 0. : m_comp[Z];
}




// -----------------------------------------------------------------------------
// Addition
template <typename T>
__host__ __device__
Vector3<T> Vector3<T>::operator + ( Vector3<T> const& vec ) const
{
    return ( Vector3<T>( m_comp[X] + vec.m_comp[X],
                         m_comp[Y] + vec.m_comp[Y], 
                         m_comp[Z] + vec.m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Subtraction
template <typename T>
__host__ __device__
Vector3<T> Vector3<T>::operator - ( Vector3<T> const& vec ) const
{
    return ( Vector3<T>( m_comp[X] - vec.m_comp[X],
                         m_comp[Y] - vec.m_comp[Y], 
                         m_comp[Z] - vec.m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Multiplication by a scalar
template <typename T>
__host__ __device__
Vector3<T> Vector3<T>::operator * ( T d ) const
{
    return ( Vector3<T>( m_comp[X] * d, m_comp[Y] * d, m_comp[Z] * d ) );
}




// -----------------------------------------------------------------------------
// Division by a scalar
template <typename T>
__host__ __device__
Vector3<T> Vector3<T>::operator / ( T d ) const
{
    return ( Vector3<T>( m_comp[X] / d, m_comp[Y] / d, m_comp[Z] / d ) );
}




// -----------------------------------------------------------------------------
// Operator +=
template <typename T>
__host__ __device__
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
__host__ __device__
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
__host__ __device__
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
__host__ __device__
Vector3<T>& Vector3<T>::operator /= ( T d )
{
    m_comp[X] /= d;
    m_comp[Y] /= d;
    m_comp[Z] /= d;
    return ( *this );
}




// -----------------------------------------------------------------------------
// dot product
template <typename T>
__host__ __device__
T Vector3<T>::operator * ( Vector3<T> const& vec ) const
{
    return ( m_comp[X] * vec.m_comp[X] + 
             m_comp[Y] * vec.m_comp[Y] +
             m_comp[Z] * vec.m_comp[Z] );
}




// -----------------------------------------------------------------------------
// Cross product this x vec
template <typename T>
__host__ __device__
Vector3<T> Vector3<T>::operator ^ ( Vector3<T> const& vec ) const
{
    return ( Vector3<T>( 
        m_comp[1] * vec.m_comp[2] - m_comp[2] * vec.m_comp[1],
      - m_comp[0] * vec.m_comp[2] + m_comp[2] * vec.m_comp[0],
        m_comp[0] * vec.m_comp[1] - m_comp[1] * vec.m_comp[0] ) );
}




// -----------------------------------------------------------------------------
// ith component accessor
template <typename T>
__host__ __device__
T Vector3<T>::operator [] ( size_t i ) const
{
    return ( m_comp[i] );
}




// -----------------------------------------------------------------------------
// Equal operator to another Vector3 object
template <typename T>
__host__ __device__
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
__host__ __device__
Vector3<T> Vector3<T>::operator - () const
{
    return ( Vector3<T>( - m_comp[X], - m_comp[Y], - m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Comparison operator
template <typename T>
__host__ __device__
bool Vector3<T>::operator == ( Vector3<T> const& vec ) const
{
    return ( m_comp[X] == vec[X] && 
             m_comp[Y] == vec[Y] &&
             m_comp[Z] == vec[Z] );
}




// -----------------------------------------------------------------------------
// Difference operator
template <typename T>
__host__ __device__
bool Vector3<T>::operator != ( Vector3<T> const& vec ) const
{
    return ( m_comp[X] != vec[X] ||
             m_comp[Y] != vec[Y] ||
             m_comp[Z] != vec[Z] );
}




// -----------------------------------------------------------------------------
// Conversion operator to float
template <>
__host__ __device__
Vector3<double>::operator Vector3<float> () const
{
    return ( Vector3<float> ( (float) m_comp[X],
                              (float) m_comp[Y],
                              (float) m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Vector3<float>;
template class Vector3<double>;