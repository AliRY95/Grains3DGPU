#include "Box.hh"


// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
template <typename T>
__HOSTDEVICE__ 
Box<T>::Box( T x, 
             T y,
             T z )
: m_extent( Vector3<T>( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Constructor with a vector containing the edge half-lengths
template <typename T>
__HOSTDEVICE__
Box<T>::Box( Vector3<T> const& extent_ )
: m_extent( extent_ )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Box<T>::~Box()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Box<T>::getConvexType() const
{
    return ( BOX );
}




// -----------------------------------------------------------------------------
// Gets values of the edge length
template <typename T>
__HOSTDEVICE__
Vector3<T> Box<T>::getExtent() const
{
    return ( m_extent );
}




// -----------------------------------------------------------------------------
// Sets values of the edge length
template <typename T>
__HOSTDEVICE__
void Box<T>::setExtent( T x, T y, T z )
{
    m_extent = Vector3<T>( x, y, z );
}




// -----------------------------------------------------------------------------
// Returns the volume of the box
template <typename T>
__HOSTDEVICE__
T Box<T>::computeVolume() const
{
    return ( T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z] );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
bool Box<T>::computeInertia( T* inertia, 
                             T* inertia_1 ) const
{
    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[0] = T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[Y] * m_extent[Y] + m_extent[Z] * m_extent[Z] ) / T( 3 );
    inertia[3] = T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[X] * m_extent[X] + m_extent[Z] * m_extent[Z] ) / T( 3 );
    inertia[5] = T( 8 ) * m_extent[X] * m_extent[Y] * m_extent[Z]
    * ( m_extent[Y] * m_extent[Y] + m_extent[X] * m_extent[X] ) / T( 3 );

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[0] = T( 1 ) / inertia[0];
    inertia_1[3] = T( 1 ) / inertia[3];
    inertia_1[5] = T( 1 ) / inertia[5];

    return ( true );
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the box
template <typename T>
__HOSTDEVICE__
T Box<T>::computeCircumscribedRadius() const
{
    return ( m_extent.norm() );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to box
template <typename T>
__HOSTDEVICE__
Vector3<T> Box<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_extent[X], 
                         m_extent[Y], 
                         m_extent[Z] ) );
}




// -----------------------------------------------------------------------------
// Box support function, returns the support point P, i.e. the point on the
// surface of the box that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Box<T>::support( Vector3<T> const& v ) const
{
    T norm = v.norm2();
    if ( norm < HIGHEPS )
        return ( Vector3<T>() );
    else
        return ( Vector3<T>( v[X] < T( 0 ) ? -m_extent[X] : m_extent[X],
                             v[Y] < T( 0 ) ? -m_extent[Y] : m_extent[Y],
                             v[Z] < T( 0 ) ? -m_extent[Z] : m_extent[Z] ) );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Box<float>;
template class Box<double>;