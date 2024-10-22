#include "CollisionDetection.hh"
#include "GJK_JH.hh"
#include "GJK_AY.hh"
#include "GJK_SV.hh"
#include "OBB.hh"
#include "MiscMath.hh"
#include "MatrixMath.hh"


/* ========================================================================== */
/*                             Low-Level Methods                              */
/* ========================================================================== */
// Returns whether two rigid bodies os spherical shape intersect
template <typename T, typename U>
__HOSTDEVICE__
static INLINE
bool intersectSpheres( RigidBody<T, U> const& rbA,
                       RigidBody<T, U> const& rbB,
                       Vector3<T> const& b2a )
{
    T radiiSum = rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius();
    T dist2 = norm2( b2a );
    return ( dist2 < radiiSum * radiiSum );
}




// -----------------------------------------------------------------------------
// Returns the contact information (if any) for 2 rigid bodies of spherical 
// shape
template <typename T, typename U>
__HOSTDEVICE__
static INLINE
ContactInfo<T> closestPointsSpheres( RigidBody<T, U> const& rbA,
                                     RigidBody<T, U> const& rbB,
                                     Transform3<T> const& a2w,
                                     Transform3<T> const& b2w )
{
    T rA = rbA.getCircumscribedRadius();
    T rB = rbB.getCircumscribedRadius();
    Vector3<T> cenA = a2w.getOrigin();
    Vector3<T> vecBA = b2w.getOrigin() - cenA;
    // We calculate the overlap, and then normalize the distance vector.
    T overlap = vecBA.norm() - rA - rB;
    vecBA.normalize();
    if ( overlap < T( 0 ) )
    {
        Vector3<T> contactPt = cenA + 
                               ( rA + T( .5 ) * overlap ) * vecBA;
        Vector3<T> contactVec = overlap * vecBA;
        return( ContactInfo<T>( contactPt,
                                contactVec,
                                overlap ) );
    }
    else
        return ( noContact );
}




// -----------------------------------------------------------------------------
// Returns the contact information (if any) for 2 rigid bodies if the SECOND ONE 
// is a rectangle
template <typename T, typename U>
__HOSTDEVICE__
static INLINE
ContactInfo<T> closestPointsRectangle( RigidBody<T, U> const& rbA,
                                       RigidBody<T, U> const& rbB,
                                       Transform3<T> const& a2w,
                                       Transform3<T> const& b2w )
{
    Convex<T> const* convexA = rbA.getConvex();
    Convex<T> const* convexB = rbB.getConvex();

    // rectangle center
    Vector3<T> const rPt = b2w.getOrigin();
    // rectangle normal is b2w.getBasis() * [0, 0, 1] which is the last column
    // of the transformation matrix
    Vector3<T> rNorm( b2w.getBasis()[X][Z], 
                      b2w.getBasis()[Y][Z],
                      b2w.getBasis()[Z][Z] ); 
    rNorm.normalized();
    rNorm = copysign( T( 1 ), rNorm * ( a2w.getOrigin() - rPt ) ) * rNorm;
    // Contact point on the particle
    Vector3<T> pointA = ( a2w ) 
                        ( convexA->support( ( -rNorm ) * a2w.getBasis() ) );
    if ( rNorm * ( pointA - rPt ) < T( 0 ) )
    {
        // The projection point on the rectangle plane
        Vector3<T> pointB = ( ( rPt - pointA ) * rNorm ) * rNorm + pointA;
        // The projection point lies on the rectangle?
        // TODO:
        // if ( ( pointB - rPt ).isInBox(  ) )
        // {
            Vector3<T> contactPt = T( 0.5 ) * ( pointA + pointB );
            Vector3<T> contactVec = pointB - pointA;
            T overlap = - norm( contactVec );
            return ( ContactInfo<T>( contactPt,
                                     contactVec,
                                     overlap ) );
        // }
    }
    else
        return ( noContact );
}




/* ========================================================================== */
/*                            High-Level Methods                              */
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
        return( intersectSpheres<T>( rbA,
                                     rbB,
                                     b2w.getOrigin() - a2w.getOrigin() ) );
    }

    // General case
    U overlap = norm( b2w.getOrigin() - a2w.getOrigin() ) - 
                ( rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() );
    if ( overlap < U( 0 ) )
    {
        // Bounding volume test
        // If the types T and U are not the same, we must cast transformations
        // from type T to type U.
        bool preCollision = true;
        if constexpr ( std::is_same<T, U>::value )
            preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                a2w,
                                                b2w );
        else
            preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                Transform3<U>( a2w ),
                                                Transform3<U>( b2w ) );

        if ( preCollision )
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
        return( intersectSpheres<T>( rbA,
                                     rbB,
                                     b2a.getOrigin() ) );
    }

    // General case
    U overlap = norm( b2a.getOrigin() ) -
                ( rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() );
    if ( overlap < U( 0 ) )
    {
        // Bounding volume test
        bool preCollision = true;
        // If the types T and U are not the same, we must cast transformations
        // from type T to type U.
        if constexpr ( std::is_same<T, U>::value )
            preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                b2a );
        else
            preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                Transform3<U>( b2a ) );

        if ( preCollision )
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
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        return ( closestPointsSpheres<T>( rbA,
                                          rbB,
                                          a2w,
                                          b2w ) );
    }
    else if ( convexB->getConvexType() == RECTANGLE )
    {
        return( closestPointsRectangle<T>( rbA,
                                           rbB,
                                           a2w,
                                           b2w ) );
    }

    
    // General case
    U overlap = norm( b2w.getOrigin() - a2w.getOrigin() ) - 
                ( rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() );
    if ( overlap < U( 0 ) )
    {
        // General case
        // Bounding volume test
        // If the types T and U are not the same, we must cast transformations
        // from type T to type U.
        bool preCollision = true;
        if constexpr ( std::is_same<T, U>::value )
            preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                a2w,
                                                b2w );
        else
            preCollision = intersectOrientedBoundingBox( 
                                                *( rbA.getBoundingBox() ), 
                                                *( rbB.getBoundingBox() ),
                                                Transform3<U>( a2w ),
                                                Transform3<U>( b2w ) );
        
        if ( preCollision )
        {
            // TODO
            Transform3<T> a2wCrust( a2w );
            a2wCrust.composeWithScaling( rbA.getScalingVector() );
            Transform3<T> b2wCrust( b2w );
            b2wCrust.composeWithScaling( rbB.getScalingVector() );
            //
            Vector3<T> ptA, ptB;
            int nbIterGJK = 0;
            T distance = computeClosestPoints_GJK_SV( *convexA, 
                                                      *convexB,
                                                      a2wCrust,
                                                      b2wCrust,
                                                      ptA,
                                                      ptB,
                                                      nbIterGJK );
            
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
            contactVec.round();
            contactVec *= - distance;

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