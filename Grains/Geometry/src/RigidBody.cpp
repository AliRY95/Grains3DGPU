#include "GJK.hh"
// #include "GJK_SV.hh"
#include "OBB.hh"
#include "RigidBody.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T>
__HOSTDEVICE__
RigidBody<T>::RigidBody()
: m_convex( NULL )
, m_crustThickness( T( 0 ) )
, m_boundingBox( NULL )
{}




// -----------------------------------------------------------------------------
// Constructor with a convex and the crust thickness
template <typename T>
__HOSTDEVICE__
RigidBody<T>::RigidBody( Convex<T>* convex, 
                         T ct )
: m_convex( convex )
, m_crustThickness( ct )
{
    m_volume = m_convex->computeVolume();
    // bool tmp = m_convex->computeInertia( m_inertia, m_inertia_1 );
    m_boundingBox = new BoundingBox( m_convex->computeBoundingBox() );
    m_circumscribedRadius = m_convex->computeCircumscribedRadius();
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T>
__HOSTDEVICE__
RigidBody<T>::~RigidBody()
{
    delete m_convex;
    delete m_boundingBox;
}




// -----------------------------------------------------------------------------
// Returns the rigid body's convex
template <typename T>
__HOSTDEVICE__
Convex<T>* RigidBody<T>::getConvex() const
{
    return ( m_convex );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's crust thickness
template <typename T>
__HOSTDEVICE__
T RigidBody<T>::getCrustThickness() const
{
    return ( m_crustThickness );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's volume
template <typename T>
__HOSTDEVICE__
T RigidBody<T>::getVolume() const
{
    return ( m_volume );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's inertia
template <typename T>
__HOSTDEVICE__
T* RigidBody<T>::getInertia() const
{
    return ( m_inertia );
}




// -----------------------------------------------------------------------------
// Returns the inverse of rigid body's inertia
template <typename T>
__HOSTDEVICE__
T* RigidBody<T>::getInertia_1() const
{
    return ( m_inertia_1 );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's bounding box
template <typename T>
__HOSTDEVICE__
BoundingBox<T>* RigidBody<T>::getBoundingBox() const
{
    return ( m_boundingBox );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's circumscribed radius
template <typename T>
__HOSTDEVICE__
T RigidBody<T>::getCircumscribedRadius() const
{
    return ( m_circumscribedRadius );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class RigidBody<float>;
template class RigidBody<double>;




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Returns whether 2 rigid bodies intersect
template <typename T>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T> const& rbA,
                           RigidBody<T> const& rbB,
                           Transform3<T> const& a2w,
                           Transform3<T> const& b2w )
{
    Convex<T> const* convexA = rbA.getConvex();
    Convex<T> const* convexB = rbB.getConvex();

    // In case the 2 rigid bodies are spheres
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        T radiiSum = rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius();
        T dist2 = ( a2w.getOrigin() - b2w.getOrigin() ).norm2();
        return ( dist2 < radiiSum * radiiSum );
    }

    // General case
    Vector3<T> cenA = a2w.getOrigin();
    Vector3<T> cenB = b2w.getOrigin();
    T dist = ( cenB - cenA ).norm();
    if ( dist < rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() )
    {
        if ( intersectOrientedBoundingBox( *( rbA.getBoundingBox() ), 
                                           *( rbB.getBoundingBox() ),
                                           a2w, 
                                           b2w ) )
            return( intersectGJK( *convexA, 
                                  *convexB,
                                  a2w, 
                                  b2w ) );
    }
    return ( false );
}




// -----------------------------------------------------------------------------
// Returns whether 2 rigid bodies intersect using the GJK algorithm - relative
// transformation
template <typename T>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T> const& rbA,
                           RigidBody<T> const& rbB,
                           Transform3<T> const& b2a )
{
    Convex<T> const* convexA = rbA.getConvex();
    Convex<T> const* convexB = rbB.getConvex();

    // In case the 2 rigid bodies are spheres
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        T radiiSum = rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius();
        T dist2 = ( b2a.getOrigin() ).norm2();
        return ( dist2 < radiiSum * radiiSum );
    }

    // General case
    Vector3<T> posB2A = b2a.getOrigin();
    T dist = posB2A.norm();
    if ( dist < rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() )
    {
        if ( intersectOrientedBoundingBox( *( rbA.getBoundingBox() ), 
                                           *( rbB.getBoundingBox() ),
                                           b2a ) )
            return( intersectGJK( *convexA, 
                                  *convexB,
                                  b2a ) );
    }
    return ( false );
}




// -----------------------------------------------------------------------------
// Returns the contact information (if any) for 2 rigid bodies
template <typename T>
__HOSTDEVICE__
ContactInfo<T> closestPointsRigidBodies( RigidBody<T> const& rbA,
                                         RigidBody<T> const& rbB,
                                         Transform3<T> const& a2w,
                                         Transform3<T> const& b2w )
{
    Convex<T> const* convexA = rbA.getConvex();
    Convex<T> const* convexB = rbB.getConvex();

    // General case
    Vector3<T> cenA = a2w.getOrigin();
    Vector3<T> cenB = b2w.getOrigin();
    T dist = ( cenB - cenA ).norm();
    if ( dist < rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() )
    {
        // TODO: RECTANGLE

        // TODO: SPHERE
        // In case the 2 rigid bodies are spheres
        // if ( convexA->getConvexType() == SPHERE && 
        //      convexB->getConvexType() == SPHERE )
            // return ( sphericalContact( *this, neighbor ) );

        // General case
        if ( intersectOrientedBoundingBox( *( rbA.getBoundingBox() ), 
                                           *( rbB.getBoundingBox() ),
                                           a2w,
                                           b2w ) )
        {
            int nbIterGJK = 0;
            Vector3<T> ptA, ptB;
            T distance = closestPointsGJK( *convexA, 
                                           *convexB,
                                           a2w,
                                           b2w,
                                           ptA,
                                           ptB,
                                           nbIterGJK );
            // T distance = compute_minimum_distance( *convexA, 
            //                                     *convexB,
            //                                     a2w,
            //                                     b2w,
            //                                     ptA,
            //                                     ptB,
            //                                     nbIterGJK );
            // TODO: ERROR HANDLING
            
            // Sum of crust thicknesses
            T ctSum = rbA.getCrustThickness() + rbB.getCrustThickness();

            // Points A and B are in their respective local coordinate systems
            // Thus we transform them into the world coordinate system
            ptA = (a2w)( ptA );
            ptB = (b2w)( ptB );

            // Contact point definition as the mid point between ptA and ptB
            Vector3<T> contactPt = ( ptA + ptB ) / 2.;

            // Computation of the actual overlap vector
            // If contact, crustA + crustB - distance > 0, the overlap vector is
            // directed from B to A
            // If no contact, crustA + crustB - distance < 0 and we do not care 
            // about the direction of the overlap vector
            Vector3<T> contactVec = ( ptA - ptB ) / distance;
            contactVec.round();
            contactVec *= ctSum - distance;

            // Computation of the actual overlap 
            // distance = distance - crustA - crustB
            // If actual overlap distance < 0 => contact otherwise no contact
            distance -= ctSum;

            return ( ContactInfo<T>( contactPt, contactVec, distance ) );
        }
        else
            return ( ContactInfo<T>( Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ), 
                                     Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ),
                                     1.e20 ) );
    }
    else
        return ( ContactInfo<T>( Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ), 
                                 Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ),
                                 1.e20 ) );
}