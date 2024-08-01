#include "CollisionDetection.hh"
#include "MiscMath.hh"
#include "MatrixMath.hh"
#include "OBB.hh"
#include "GJK.hh"
#include "GJK_SV.hh"
#include "openGJK.hh"


// -----------------------------------------------------------------------------
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
                                           Transform3<U>( a2w ),
                                           Transform3<U>( b2w ) ) )
            return( intersectGJK( *convexA, 
                                  *convexB,
                                  a2w, 
                                  b2w ) );
    }
    return ( false );
}




// -----------------------------------------------------------------------------
// Returns whether 2 rigid bodies intersect
template <typename T>
__HOSTDEVICE__
bool intersectRigidBodies( RigidBody<T, T> const& rbA,
                           RigidBody<T, T> const& rbB,
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
            return ( ContactInfo<T>( zeroVector3T, 
                                     zeroVector3T,
                                     T( 0 ) ) );
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
            T distance = closestPointsGJK( *convexA, 
                                           *convexB,
                                           a2w,
                                           b2w,
                                           ptA,
                                           ptB,
                                           nbIterGJK );
            // T distance = closestPointsGJK_SV2( *convexA, 
            //                                   *convexB,
            //                                   a2w,
            //                                   b2w,
            //                                   ptA,
            //                                   ptB,
            //                                   nbIterGJK );
            
            // TODO: ERROR HANDLING
            
            // Sum of crust thicknesses
            T ctSum = rbA.getCrustThickness() + rbB.getCrustThickness();

            // Computation of the actual overlap 
            // distance = distance - crustA - crustB
            // If actual overlap distance < 0 => contact otherwise no contact
            distance -= ctSum;
            if ( distance > T( 0 ) )
                return( noContact );

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
            Vector3<T> contactVec = ptA - ptB;
            contactVec.normalize();

            return ( ContactInfo<T>( contactPt, contactVec, distance ) );
        }
    }
    
    return ( noContact );
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
            return ( noContact );
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
            T distance = closestPointsGJK( *convexA, 
                                           *convexB,
                                           a2w,
                                           b2w,
                                           ptA,
                                           ptB,
                                           nbIterGJK );
            // T distance = closestPointsGJK_SV2( *convexA, 
            //                                   *convexB,
            //                                   a2w,
            //                                   b2w,
            //                                   ptA,
            //                                   ptB,
            //                                   nbIterGJK );
            
            // TODO: ERROR HANDLING

            // Sum of crust thicknesses
            T ctSum = rbA.getCrustThickness() + rbB.getCrustThickness();
            
            // Computation of the actual overlap 
            // distance = distance - crustA - crustB
            // If actual overlap distance < 0 => contact otherwise no contact
            distance -= ctSum;
            if ( distance > T( 0 ) )
                return( noContact );

            // Points A and B are in their respective local coordinate systems
            // Thus we transform them into the world coordinate system
            ptA = (a2w)( ptA );
            ptB = (b2w)( ptB );

            // Contact point definition as the mid point between ptA and ptB
            Vector3<T> contactPt = ( ptA + ptB ) / T( 2 );

            // Computation of the actual overlap vector
            // the overlap vector is directed from B to A
            Vector3<T> contactVec = ptA - ptB;
            contactVec.normalize();

            return ( ContactInfo<T>( contactPt, contactVec, distance ) );
        }
    }
    
    return ( noContact );
}




// -----------------------------------------------------------------------------
// Explicit instantiation
#define X( T, U ) \
template                                                                       \
__HOSTDEVICE__                                                                 \
bool intersectRigidBodies( RigidBody<T, U> const& rbA,                         \
                           RigidBody<T, U> const& rbB,                         \
                           Transform3<T> const& a2w,                           \
                           Transform3<T> const& b2w );                         \
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