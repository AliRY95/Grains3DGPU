#include "Convex.hh"
#include "Sphere.hh"


// -----------------------------------------------------------------------------
// Constructor with radius
template <typename T>
__HOSTDEVICE__ 
Sphere<T>::Sphere( T r )
: m_radius( r )
{}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
Sphere<T>::~Sphere()
{}




// -----------------------------------------------------------------------------
// Returns the convex type
template <typename T>
__HOSTDEVICE__
ConvexType Sphere<T>::getConvexType() const
{
    return ( SPHERE );
}




// -----------------------------------------------------------------------------
// Returns the radius
template <typename T>
__HOSTDEVICE__
T Sphere<T>::getRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Sets the radius
template <typename T>
__HOSTDEVICE__
void Sphere<T>::setRadius( T r )
{
    m_radius = r;
}




// -----------------------------------------------------------------------------
// Returns the volume of the Sphere
template <typename T>
__HOSTDEVICE__
T Sphere<T>::computeVolume() const
{
    return ( T( 4 ) * M_PI * m_radius * m_radius * m_radius / T ( 3 ) );
}




// -----------------------------------------------------------------------------
// Computes the inertia tensor and the inverse of the inertia tensor
template <typename T>
__HOSTDEVICE__
bool Sphere<T>::computeInertia( T* inertia, 
                                        T* inertia_1 ) const
{
    inertia[1] = inertia[2] = inertia[4] = T( 0 );
    inertia[5] = inertia[3] = inertia[0] = T( 8 ) * M_PI / T( 15 ) * 
                                                        pow( m_radius, T( 5 ));

    inertia_1[1] = inertia_1[2] = inertia_1[4] = T( 0 );
    inertia_1[5] = inertia_1[3] = inertia_1[0] = T( 1 ) / inertia[0];

    return ( true );
}




// -----------------------------------------------------------------------------
// Returns the circumscribed radius of the Sphere
template <typename T>
__HOSTDEVICE__
T Sphere<T>::computeCircumscribedRadius() const
{
    return ( m_radius );
}




// -----------------------------------------------------------------------------
// Returns the bounding box to Sphere
template <typename T>
__HOSTDEVICE__
Vector3<T> Sphere<T>::computeBoundingBox() const
{
    return ( Vector3<T>( m_radius, 
                         m_radius, 
                         m_radius ) );
}




// -----------------------------------------------------------------------------
// Sphere support function, returns the support point P, i.e. the point on the
// surface of the Sphere that satisfies max(P.v)
template <typename T>
__HOSTDEVICE__
Vector3<T> Sphere<T>::support( Vector3<T> const& v ) const
{
    T norm = v.norm2();
    if ( norm < HIGHEPS )
        return ( Vector3<T>() );
    else
    {
        T r = m_radius / norm;
        return ( Vector3<T>( v[X] * r, v[Y] * r, v[Z] * r ) );
    }
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class Sphere<float>;
template class Sphere<double>;