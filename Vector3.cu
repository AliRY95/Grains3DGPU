#ifndef _VECTOR3_CUH_
#define _VECTOR3_CUH_

#include "Basic.cuh"


// =============================================================================
/** @brief The class Vector3.

    Vector/Point in a 3D space.

    @author A.Yazdani - 2023 - Construction */
// =============================================================================
template <typename T>
class Vector3
{
  protected:
    /**@name Parameters */
    //@{
    T m_comp[3]; /**< array of 3 components */
    //@}

  public:
    /**@name Constructors */
    //@{
    /** @brief Default constructor
    @param def value of all 3 components */
    __host__ __device__ Vector3( T def = T() );

    /** @brief Constructor with 3 components as inputs
    @param x 1st component
    @param y 2nd component
    @param z 3rd component*/
    __host__ __device__ Vector3( T x, T y, T z );

    /** @brief Copy constructor
    @param g copied Vector3 object */
    __host__ __device__ Vector3( Vector3<T> const& vec );

    /** @brief Destructor */
    __host__ __device__ ~Vector3();
    //@}


    /** @name Methods */
    //@{
    /** @brief Sets the components */
    __host__ __device__ void setValue( T x, T y, T z );

    /** @brief Unitary nomalization operator */
    __host__ __device__ void normalize();

    /** @brief Returns a vector corresponding to the normalized vector */
    __host__ __device__ Vector3<T> normalized() const;

    /** @brief Returns the norm of the vector
    @param vec the Vector3 object */
    __host__ __device__ T Norm() const;
    //@}


    /** @name Operators */
    //@{
    /** @brief Addition
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T> operator + ( Vector3<T> const& vec ) const;

    /** @brief Subtraction
    @param vec 2nd object */
    __host__ __device__ Vector3<T> operator - ( Vector3<T> const& vec ) const;
    
    /** @brief Multiplication by a scalar
    @param d multiplication factor */
    __host__ __device__ Vector3<T> operator * ( T d ) const;
    
    /** @brief Division by a scalar
    @param d division factor */
    __host__ __device__ Vector3<T> operator / ( T d ) const;

    /** @brief Operator +=
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T>& operator += ( Vector3<T> const& vec );

    /** @brief Operator -=
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T>& operator -= ( Vector3<T> const& vec );
    
    /** @brief Unitary operator *= by a scalar
    @param d multiplication factor */
    __host__ __device__ Vector3<T>& operator *= ( T d );

    /** @brief Unitary operator /= by a scalar
    @param d division factor */
    __host__ __device__ Vector3<T>& operator /= ( T d );

    /** @brief dot product
    @param vec 2nd Vector3 object */
    __host__ __device__ T operator * ( Vector3<T> const& vec ) const;
    
    /** @brief Cross product this x rhv
    @param vec 2nd Vector3 object */
    __host__ __device__ Vector3<T> operator ^ ( Vector3<T> const& vec ) const;

    /** @brief ith component accessor
    @param i component index */
    __host__ __device__ T operator [] ( size_t i ) const;

    /** @brief Equal operator to another Vector3 object
    @param vec the other Vector3 object */
    __host__ __device__ Vector3<T>& operator = ( Vector3<T> const& vec );    

    /** @brief Unitary operator -. Returns an object with negative
    components */
    __host__ __device__ Vector3<T> operator - () const;

    /** @brief Comparaison operator
    @param vec 2nd Vector3 object */
    __host__ __device__ bool operator == ( Vector3<T> const& vec ) const;

    /** @brief Difference operator
    @param vec 2nd Vector3 object */
    __host__ __device__ bool operator != ( Vector3<T> const& vec ) const;
    //@}

    /** @name Friend methods */
    //@{
    /** @brief Returns whether the vector norm is less than EPSILON2
    where EPSILON2 is defined in Basic.H
    @param v Vector3 object */
    template <typename Y>
    __host__ __device__ friend bool approxZero( Vector3<Y> const& v );

    /** @brief Returns the norm of the vector
    @param v the Vector3 object */
    template <typename Y>
    __host__ __device__ friend Y Norm( Vector3<Y> const& v );

    /** @brief Returns the norm square of the vector
    @param v the Vector3 object */
    template <typename Y>
    __host__ __device__ friend Y Norm2( Vector3<Y> const& v );
    //@}
};


// /** @name Vector3 : External methods */
// //@{
// /** @brief Returns the norm of the vector
//  @param v vector */
// template <typename T>
// __host__ __device__ T Norm( Vector3<T> const& v );

// /** @brief Returns the squared norm of the vector
//  @param v vector */
// template <typename T>
// __host__ __device__ T Norm2( Vector3<T> const& v );

// /** @brief Returns whether the vector norm is less than EPSILON2
// where EPSILON2 is defined in Basic.H
//  @param v Vector3 object */
//  template <typename T>
// __host__ __device__ bool approxZero( Vector3<T> const& v );
// //@}


// static Vector3 Vector3Nul; /**< Vector3 nul (0.,0.,0.)  */


/* ========================================================================== */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */


// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__host__ __device__ Vector3<T>::Vector3( T def )
{
  m_comp[X] = m_comp[Y] = m_comp[Z] = def;
}




// -----------------------------------------------------------------------------
// Constructor with 3 components as inputs
template <typename T>
__host__ __device__ Vector3<T>::Vector3( T x, T y, T z )
{
  m_comp[X] = x;
  m_comp[Y] = y;
  m_comp[Z] = z;
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__host__ __device__ Vector3<T>::Vector3( Vector3<T> const& vec )
{
  m_comp[X] = vec.m_comp[X];
  m_comp[Y] = vec.m_comp[Y];
  m_comp[Z] = vec.m_comp[Z];
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__ Vector3<T>::~Vector3()
{}




// -----------------------------------------------------------------------------
/* Sets the components */
template <typename T>
__host__ __device__ void Vector3<T>::setValue( T x, T y, T z )
{
  m_comp[X] = x;
  m_comp[Y] = y;
  m_comp[Z] = z;
}




// -----------------------------------------------------------------------------
// Unitary nomalization operator
template <typename T>
__host__ __device__ void Vector3<T>::normalize()
{
  *this /= this->Norm();
}




// -----------------------------------------------------------------------------
// Returns a vector corresponding to the normalized vector
template <typename T>
__host__ __device__ Vector3<T> Vector3<T>::normalized() const
{
  return ( *this / this->Norm() );
}




// -----------------------------------------------------------------------------
//Returns the norm of the vector
template <typename T>
__host__ __device__ T Vector3<T>::Norm() const
{
  return ( sqrt( m_comp[X] * m_comp[X] + 
                 m_comp[Y] * m_comp[Y] + 
                 m_comp[Z] * m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Addition
template <typename T>
__host__ __device__ Vector3<T> Vector3<T>::operator + ( Vector3<T> const& vec ) 
                                                                          const
{
  return ( Vector3<T>( m_comp[X] + vec.m_comp[X],
                       m_comp[Y] + vec.m_comp[Y], 
                       m_comp[Z] + vec.m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Subtraction
template <typename T>
__host__ __device__ Vector3<T> Vector3<T>::operator - ( Vector3<T> const& vec ) 
                                                                          const
{
  return ( Vector3<T>( m_comp[X] - vec.m_comp[X],
                       m_comp[Y] - vec.m_comp[Y], 
                       m_comp[Z] - vec.m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Multiplication by a scalar
template <typename T>
__host__ __device__ Vector3<T> Vector3<T>::operator * ( T d ) const
{
  return ( Vector3<T>( m_comp[X] * d, m_comp[Y] * d, m_comp[Z] * d ) );
}




// -----------------------------------------------------------------------------
// Division by a scalar
template <typename T>
__host__ __device__ Vector3<T> Vector3<T>::operator / ( T d ) const
{
  return ( Vector3<T>( m_comp[X] / d, m_comp[Y] / d, m_comp[Z] / d ) );
}




// -----------------------------------------------------------------------------
// Operator +=
template <typename T>
__host__ __device__ Vector3<T>& Vector3<T>::operator += ( Vector3<T> const& vec )
{
  m_comp[X] += vec.m_comp[X];
  m_comp[Y] += vec.m_comp[Y];
  m_comp[Z] += vec.m_comp[Z];
  return ( *this );
}




// -----------------------------------------------------------------------------
// Operator -=
template <typename T>
__host__ __device__ Vector3<T>& Vector3<T>::operator -= ( Vector3<T> const& vec )
{
  m_comp[X] -= vec.m_comp[X];
  m_comp[Y] -= vec.m_comp[Y];
  m_comp[Z] -= vec.m_comp[Z];
  return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator *= by a scalar
template <typename T>
__host__ __device__ Vector3<T>& Vector3<T>::operator *= ( T d )
{
  m_comp[X] *= d;
  m_comp[Y] *= d;
  m_comp[Z] *= d;
  return ( *this );
}




// -----------------------------------------------------------------------------
// Unitary operator /= by a scalar
template <typename T>
__host__ __device__ Vector3<T>& Vector3<T>::operator /= ( T d )
{
  m_comp[X] /= d;
  m_comp[Y] /= d;
  m_comp[Z] /= d;
  return ( *this );
}




// -----------------------------------------------------------------------------
// dot product
template <typename T>
__host__ __device__ T Vector3<T>::operator * ( Vector3<T> const& vec ) const
{
  return ( m_comp[X] * vec.m_comp[X] + 
           m_comp[Y] * vec.m_comp[Y] +
           m_comp[Z] * vec.m_comp[Z] );
}




// -----------------------------------------------------------------------------
// Cross product this x vec
template <typename T>
__host__ __device__ Vector3<T> Vector3<T>::operator ^ ( Vector3<T> const& vec ) 
                                                                          const
{
  return ( Vector3<T>( m_comp[1] * vec.m_comp[2] - m_comp[2] * vec.m_comp[1],
                     - m_comp[0] * vec.m_comp[2] + m_comp[2] * vec.m_comp[0],
                       m_comp[0] * vec.m_comp[1] - m_comp[1] * vec.m_comp[0] ) );
}




// -----------------------------------------------------------------------------
// ith component accessor
template <typename T>
__host__ __device__ T Vector3<T>::operator [] ( size_t i ) const
{
  return ( m_comp[i] );
}




// -----------------------------------------------------------------------------
// Equal operator to another Vector3 object
template <typename T>
__host__ __device__ Vector3<T>& Vector3<T>::operator = ( Vector3<T> const& vec )
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
__host__ __device__ Vector3<T> Vector3<T>::operator - () const
{
  return ( Vector3<T>( - m_comp[X], - m_comp[Y], - m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Comparison operator
template <typename T>
__host__ __device__ bool Vector3<T>::operator == ( Vector3<T> const& vec ) const
{
  return ( m_comp[X] == vec[X] && m_comp[Y] == vec[Y] && m_comp[Z] == vec[Z] );
}




// -----------------------------------------------------------------------------
// Difference operator
template <typename T>
__host__ __device__ bool Vector3<T>::operator != ( Vector3<T> const& vec ) const
{
  return ( m_comp[X] != vec[X] || m_comp[Y] != vec[Y] || m_comp[Z] != vec[Z] );
}




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
template <typename T>
__host__ __device__ bool approxZero( Vector3<T> const& v )
{
    return ( fabs( v.m_comp[X] ) < EPSILON1 && 
             fabs( v.m_comp[Y] ) < EPSILON1 &&
             fabs( v.m_comp[Z] ) < EPSILON1 );
}



// -----------------------------------------------------------------------------
// Returns the norm of the vector
template <typename T>
__host__ __device__ T Norm( Vector3<T> const& v )
{
  return ( sqrt( v.m_comp[X] * v.m_comp[X] + 
                 v.m_comp[Y] * v.m_comp[Y] + 
                 v.m_comp[Z] * v.m_comp[Z] ) );
}




// -----------------------------------------------------------------------------
// Returns the squared norm of the vector
template <typename T>
__host__ __device__ T Norm2( Vector3<T> const& v )
{
  return ( v.m_comp[X] * v.m_comp[X] + 
           v.m_comp[Y] * v.m_comp[Y] + 
           v.m_comp[Z] * v.m_comp[Z] );
}

#endif
