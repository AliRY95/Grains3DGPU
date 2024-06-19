#include "MatrixMath.hh"
#include "OBB.hh"
#include "GJK.hh"
#include "GJK_SV.hh"
#include "openGJK.hh"
#include "RigidBody.hh"

// -----------------------------------------------------------------------------
// Default constructor
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody()
: m_convex( NULL )
, m_crustThickness( T( 0 ) )
, m_boundingBox( NULL )
{}




// -----------------------------------------------------------------------------
// Constructor with a convex and the crust thickness
// TODO: inertia
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::RigidBody( Convex<T>* convex, 
                            T ct )
: m_convex( convex )
, m_crustThickness( ct )
{
    m_volume = m_convex->computeVolume();
    // bool tmp = m_convex->computeInertia( m_inertia, m_inertia_1 );
    // We cast type T to U just in case they are different.
    // It happens only at the start when the rigid body is created.
    m_boundingBox = new 
                    BoundingBox( Vector3<U>( m_convex->computeBoundingBox() ) );
    m_circumscribedRadius = U( m_convex->computeCircumscribedRadius() );
}




// -----------------------------------------------------------------------------
// Destructor
template <typename T, typename U>
__HOSTDEVICE__
RigidBody<T, U>::~RigidBody()
{
    delete m_convex;
    delete m_boundingBox;
}




// -----------------------------------------------------------------------------
// Returns the rigid body's convex
template <typename T, typename U>
__HOSTDEVICE__
Convex<T>* RigidBody<T, U>::getConvex() const
{
    return ( m_convex );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's crust thickness
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getCrustThickness() const
{
    return ( m_crustThickness );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's volume
template <typename T, typename U>
__HOSTDEVICE__
T RigidBody<T, U>::getVolume() const
{
    return ( m_volume );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's inertia
template <typename T, typename U>
__HOSTDEVICE__
T* RigidBody<T, U>::getInertia() const
{
    return ( m_inertia );
}




// -----------------------------------------------------------------------------
// Returns the inverse of rigid body's inertia
template <typename T, typename U>
__HOSTDEVICE__
T* RigidBody<T, U>::getInertia_1() const
{
    return ( m_inertia_1 );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's bounding box
template <typename T, typename U>
__HOSTDEVICE__
BoundingBox<U>* RigidBody<T, U>::getBoundingBox() const
{
    return ( m_boundingBox );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's circumscribed radius
template <typename T, typename U>
__HOSTDEVICE__
U RigidBody<T, U>::getCircumscribedRadius() const
{
    return ( m_circumscribedRadius );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
template class RigidBody<float, float>;
template class RigidBody<double, float>;
template class RigidBody<double, double>;




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Returns whether 2 rigid bodies intersect
template <typename T, typename U>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T, U> const& rbA,
                           RigidBody<T, U> const& rbB,
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
template <typename T, typename U>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T, U> const& rbA,
                           RigidBody<T, U> const& rbB,
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
template <typename T, typename U>
__HOSTDEVICE__
ContactInfo<T> closestPointsRigidBodies( RigidBody<T, U> const& rbA,
                                         RigidBody<T, U> const& rbB,
                                         Transform3<T> const& a2w,
                                         Transform3<T> const& b2w )
{
    // Comment on the direction of the overlap vector
    // Assuming A and B are the centers of the 2 convex bodies
    // overlap_vector = overlap * Vector3(A to B)
    // If contact, overlap is negative and overlap_vector is from B to A
    // If no contact, overlap is positive and we do not care about the direction
    // of overlap_vector

    Convex<T> const* convexA = rbA.getConvex();
    Convex<T> const* convexB = rbB.getConvex();
    
    // In case the 2 rigid bodies are spheres
        // TODO: is_same should be removed
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        Vector3<T> cenA = a2w.getOrigin();
        Vector3<T> cenB = b2w.getOrigin();
        T rA = rbA.getCircumscribedRadius();
        T rB = rbB.getCircumscribedRadius();
        Vector3<T> vecBA = cenB - cenA;
        T dist = vecBA.norm();
        T overlap = dist - rA - rB;
        if ( overlap < T( 0 ) )
        {
            Vector3<T> contactPt = cenA + 
                             ( T( .5 ) * (rA - rB) / dist + T( .5 ) ) * vecBA;
            Vector3<T> contactVec = ( overlap / dist ) * vecBA;
            return( ContactInfo<T>( contactPt,
                                    contactVec,
                                    overlap ) );
        }
        else
            return ( ContactInfo<T>( Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ), 
                                     Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ),
                                     1.e20 ) );
    }

    
    // General case
    Vector3<U> cenA = a2w.getOrigin();
    Vector3<U> cenB = b2w.getOrigin();
    U rA = rbA.getCircumscribedRadius();
    U rB = rbB.getCircumscribedRadius();
    Vector3<U> vecBA = cenB - cenA;
    U dist = vecBA.norm();
    U overlap = dist - rA - rB;
    if ( overlap < U( 0 ) )
    {
        // TODO: RECTANGLE


        // General case
        bool preCollision = true;
        preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                Transform3<U>( a2w ),
                                                Transform3<U>( b2w ) );
        
        if ( preCollision )
        {
            int nbIterGJK = 0;
            Vector3<T> ptA, ptB;
            // T distance = closestPointsGJK( *convexA, 
            //                                *convexB,
            //                                a2w,
            //                                b2w,
            //                                ptA,
            //                                ptB,
            //                                nbIterGJK );
            T distance = closestPointsGJK_SV( *convexA, 
                                              *convexB,
                                              a2w,
                                              b2w,
                                              ptA,
                                              ptB,
                                              nbIterGJK );
            // TODO: ERROR HANDLING
            // printf( "%d\n", nbIterGJK );
            // Sum of crust thicknesses
            T ctSum = rbA.getCrustThickness() + rbB.getCrustThickness();

            // Points A and B are in their respective local coordinate systems
            // Thus we transform them into the world coordinate system
            ptA = (a2w)( ptA );
            ptB = (b2w)( ptB );

            // Contact point definition as the mid point between ptA and ptB
            Vector3<T> contactPt = ( ptA + ptB ) / T( 2 );

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




// -----------------------------------------------------------------------------
// Returns the contact information (if any) for 2 rigid bodies
template <typename T>
__HOSTDEVICE__
ContactInfo<T> closestPointsRigidBodies( RigidBody<T, T> const& rbA,
                                         RigidBody<T, T> const& rbB,
                                         Transform3<T> const& a2w,
                                         Transform3<T> const& b2w )
{
    // Comment on the direction of the overlap vector
    // Assuming A and B are the centers of the 2 convex bodies
    // overlap_vector = overlap * Vector3(A to B)
    // If contact, overlap is negative and overlap_vector is from B to A
    // If no contact, overlap is positive and we do not care about the direction
    // of overlap_vector

    Convex<T> const* convexA = rbA.getConvex();
    Convex<T> const* convexB = rbB.getConvex();
    
    // In case the 2 rigid bodies are spheres
        // TODO: is_same should be removed
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        Vector3<T> cenA = a2w.getOrigin();
        Vector3<T> cenB = b2w.getOrigin();
        T rA = rbA.getCircumscribedRadius();
        T rB = rbB.getCircumscribedRadius();
        Vector3<T> vecBA = cenB - cenA;
        T dist = vecBA.norm();
        T overlap = dist - rA - rB;
        if ( overlap < T( 0 ) )
        {
            Vector3<T> contactPt = cenA + 
                             ( T( .5 ) * (rA - rB) / dist + T( .5 ) ) * vecBA;
            Vector3<T> contactVec = ( overlap / dist ) * vecBA;
            return( ContactInfo<T>( contactPt,
                                    contactVec,
                                    overlap ) );
        }
        else
            return ( ContactInfo<T>( Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ), 
                                     Vector3<T>( T( 0 ), T( 0 ), T( 0 ) ),
                                     1.e20 ) );
    }

    
    // General case
    Vector3<T> cenA = a2w.getOrigin();
    Vector3<T> cenB = b2w.getOrigin();
    T rA = rbA.getCircumscribedRadius();
    T rB = rbB.getCircumscribedRadius();
    Vector3<T> vecBA = cenB - cenA;
    T dist = vecBA.norm();
    T overlap = dist - rA - rB;
    if ( overlap < T( 0 ) )
    {
        // TODO: RECTANGLE


        // General case
        bool preCollision = true;
        preCollision = intersectOrientedBoundingBox( *( rbA.getBoundingBox() ), 
                                                     *( rbB.getBoundingBox() ),
                                                     a2w,
                                                     b2w );
        if ( preCollision )
        {
            int nbIterGJK = 0;
            Vector3<T> ptA, ptB;
            // T distance = closestPointsGJK( *convexA, 
            //                                *convexB,
            //                                a2w,
            //                                b2w,
            //                                ptA,
            //                                ptB,
            //                                nbIterGJK );
            T distance = closestPointsGJK_SV( *convexA, 
                                              *convexB,
                                              a2w,
                                              b2w,
                                              ptA,
                                              ptB,
                                              nbIterGJK );
            // TODO: ERROR HANDLING
            // printf( "%d\n", nbIterGJK );
            // Sum of crust thicknesses
            T ctSum = rbA.getCrustThickness() + rbB.getCrustThickness();

            // Points A and B are in their respective local coordinate systems
            // Thus we transform them into the world coordinate system
            ptA = (a2w)( ptA );
            ptB = (b2w)( ptB );

            // Contact point definition as the mid point between ptA and ptB
            Vector3<T> contactPt = ( ptA + ptB ) / T( 2 );

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




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T, U ) \
template                                                                       \
__HOSTDEVICE__                                                                 \
ContactInfo<T> closestPointsRigidBodies( RigidBody<T, U> const& rbA,           \
                                         RigidBody<T, U> const& rbB,           \
                                         Transform3<T> const& a2w,             \
                                         Transform3<T> const& b2w );           
X( float, float )
X( double, float )
X( double, double )
#undef X
// #define X( T ) \
// template                                                                       \
// __HOSTDEVICE__                                                                 \
// ContactInfo<T> closestPointsRigidBodies( RigidBody<T, T> const& rbA,           \
//                                          RigidBody<T, T> const& rbB,           \
//                                          Transform3<T> const& a2w,             \
//                                          Transform3<T> const& b2w );           
// X( float )
// X( double )
// #undef X