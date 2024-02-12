#include "Transform3.hh"


// -----------------------------------------------------------------------------
// Default constructor. Origin is def and matrix is identity
template <typename T>
__host__ __device__ Transform3<T>::Transform3( T def )
{
  m_basis.setValue( T( 1 ), T( 0 ), T( 0 ), 
                    T( 0 ), T( 1 ), T( 0 ), 
                    T( 0 ), T( 0 ), T( 1 ) );
  m_origin.setValue( def, def, def );
}




// -----------------------------------------------------------------------------
// Constructor with origin coordinates as inputs and matrix is identity
template <typename T>
__host__ __device__ Transform3<T>::Transform3( T x, T y, T z )
{
  m_basis.setValue( T( 1 ), T( 0 ), T( 0 ), 
                    T( 0 ), T( 1 ), T( 0 ), 
                    T( 0 ), T( 0 ), T( 1 ) );
  m_origin.setValue( x, y, z );
}




// -----------------------------------------------------------------------------
// Constructor with a 1D array of 12 values as inputs containing the
// rotation matrix coefficients following by the origin coordinates
template <typename T>
__host__ __device__ Transform3<T>::Transform3( T const t[12] ) 
{ 
  setValue( t );    
}




// -----------------------------------------------------------------------------
// Copy constructor
template <typename T>
__host__ __device__ Transform3<T>::Transform3( Transform3<T> const& t )
{
  m_basis = t.m_basis;
  m_origin = t.m_origin;
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__host__ __device__ Transform3<T>::~Transform3()
{}




// -----------------------------------------------------------------------------
// Sets the transformation with an 1D array of 12 values as inputs 
template <typename T>
__host__ __device__ void Transform3<T>::setValue( T const t[12] ) 
{
  m_basis.setValue( t[0], t[1], t[2], 
                    t[3], t[4], t[5], 
                    t[6], t[7], t[8] );
  m_origin.setValue( t[9], t[10], t[11] );
}




// -----------------------------------------------------------------------------
// Sets the transformation to the identity
template <typename T>
__host__ __device__ void Transform3<T>::setIdentity() 
{
  m_basis.setValue( T( 1 ), T( 0 ), T( 0 ), 
                    T( 0 ), T( 1 ), T( 0 ), 
                    T( 0 ), T( 0 ), T( 1 ) );
  m_origin.setValue( T( 0 ), T( 0 ), T( 0 ) );
}




// -----------------------------------------------------------------------------
// Sets the matrix part of the transformation
template <typename T>
__host__ __device__ void Transform3<T>::setBasis( Matrix3<T> const& m )
{
  m_basis = m;
}




// -----------------------------------------------------------------------------
// Sets the matrix part of the transformation with specified rotations around 
// each principal axis
template <typename T>
__host__ __device__ void Transform3<T>::setBasis( T aX, T aY, T aZ )
{
  m_basis = Matrix3<T>( cos(aZ)*cos(aY),
                        cos(aZ)*sin(aY)*sin(aX) - sin(aZ)*cos(aX),
                        cos(aZ)*sin(aY)*cos(aX) + sin(aZ)*sin(aX),
                        sin(aZ)*cos(aY),
                        sin(aZ)*sin(aY)*sin(aX) + cos(aZ)*cos(aX),
                        sin(aZ)*sin(aY)*cos(aX) - cos(aZ)*sin(aX),
                        -sin(aY),
                        cos(aY)*sin(aX),
                        cos(aY)*cos(aX) );
}




// -----------------------------------------------------------------------------
// Sets the matrix part of the transformation with specified rotations around 
// each principal axis - single precision
template<>
__host__ __device__ void Transform3<float>::setBasis( float aX, 
                                                      float aY,
                                                      float aZ )
{
  m_basis = Mat3f( cosf(aZ)*cosf(aY),
                   cosf(aZ)*sinf(aY)*sinf(aX) - sinf(aZ)*cosf(aX),
                   cosf(aZ)*sinf(aY)*cosf(aX) + sinf(aZ)*sinf(aX),
                   sinf(aZ)*cosf(aY),
                   sinf(aZ)*sinf(aY)*sinf(aX) + cosf(aZ)*cosf(aX),
                   sinf(aZ)*sinf(aY)*cosf(aX) - cosf(aZ)*sinf(aX),
                   -sinf(aY),
                   cosf(aY)*sinf(aX),
                   cosf(aY)*cosf(aX) );
}




// -----------------------------------------------------------------------------
// Sets the origin of the transformation
template <typename T>
__host__ __device__ void Transform3<T>::setOrigin( Vector3<T> const& v ) 
{
  m_origin = v;
}




// -----------------------------------------------------------------------------
// Set the transformation to the inverse of another transformation t
template <typename T>
__host__ __device__ void Transform3<T>::setToInverseTransform( 
                                                        Transform3<T> const& t ) 
{
  m_basis = t.m_basis.inverse();
  m_origin.setValue( - m_basis[X] * t.m_origin, 
                     - m_basis[Y] * t.m_origin, 
                     - m_basis[Z] * t.m_origin );
}




// -----------------------------------------------------------------------------
// Composition of affine transformations: this = t2 o t1 (t1 first
// followed by t2)
template <typename T>
__host__ __device__ void Transform3<T>::setToTransformsComposition( 
                                                      Transform3<T> const& t1, 
                                                      Transform3<T> const& t2 ) 
{  
  m_basis = t2.m_basis * t1.m_basis;
  m_origin = t2( t1.m_origin );
}




// -----------------------------------------------------------------------------
// Returns the orientation of the transformation
template <typename T>
__host__ __device__ Matrix3<T> Transform3<T>::getBasis() const
{
  return ( m_basis ) ;
}




// -----------------------------------------------------------------------------
// Returns a pointer to the origin of the transformation
template <typename T>
__host__ __device__ Vector3<T> Transform3<T>::getOrigin() const
{
  return ( m_origin ) ;
}




// -----------------------------------------------------------------------------
// Composition with a scaling transformation: this = this o scaling
template <typename T>
__host__ __device__ void Transform3<T>::composeWithScaling( T x, T y, T z )
{
  m_basis *= Matrix3<T>( x, T( 0 ), T( 0 ),
                         T( 0 ), y, T( 0 ),
                         T( 0 ), T( 0 ), z );
} 




// -----------------------------------------------------------------------------
// Composition on the left by a rotation described by a transform:
// this = rot o this (this first followed by rot)
// This composition leaves the origin unchanged but does not check that rot 
// is indeed a rotation 
template <typename T>
__host__ __device__ void Transform3<T>::composeLeftByRotation( 
                                                        Transform3<T> const& t ) 
{
   m_basis = t.m_basis * m_basis;
}




// -----------------------------------------------------------------------------
// Composition on the left by a translation:
// this = trans(vector) o this (this first followed by trans(vector))
template <typename T>
__host__ __device__ void Transform3<T>::composeLeftByTranslation( 
                                                          Vector3<T> const& v )
{ 
  m_origin += v;
}




// -----------------------------------------------------------------------------
// Composition on the left by another affine transformation: 
// this = t o this (this first followed by t)
template <typename T>
__host__ __device__ void Transform3<T>::composeLeftByTransform( 
                                                        Transform3<T> const& t ) 
{
   m_origin = t.m_origin + t.m_basis * m_origin;
   m_basis = t.m_basis * m_basis;
}




// -----------------------------------------------------------------------------
// Composition on the right by another affine transformation: 
// this = this o t (t first followed by this)
template <typename T>
__host__ __device__ void Transform3<T>::composeRightByTransform( 
                                                        Transform3<T> const& t ) 
{
  m_origin += m_basis * t.m_origin;
  m_basis *= t.m_basis;
}




// -----------------------------------------------------------------------------
// Returns the result of applying the transformation to the input vector
template <typename T>
__host__ __device__ Vector3<T> Transform3<T>::operator ()
                                                  ( Vector3<T> const& v ) const 
{
  return ( Vector3<T>( m_basis[X] * v + m_origin[X], 
                       m_basis[Y] * v + m_origin[Y], 
                       m_basis[Z] * v + m_origin[Z] ) );
}




// -----------------------------------------------------------------------------
// Equal operator to another Transform object
template <typename T>
__host__ __device__ Transform3<T>& Transform3<T>::operator = 
                                                      ( Transform3<T> const& t )
{
  if ( &t != this )
  {  
    m_basis = t.m_basis;
    m_origin = t.m_origin;
  }
  return ( *this );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Transform3<float>;
template class Transform3<double>;