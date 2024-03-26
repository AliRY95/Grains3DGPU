#include "BoundingBox.hh"
#include "Convex.hh"
#include "ContactInfo.hh"
#include "RigidBody.hh"
#include "openGJK.hh"


// -----------------------------------------------------------------------------
// Default constructor
__host__ __device__
RigidBody::RigidBody()
: m_convex( NULL )
, m_crustThickness( 0. )
, m_boundingBox( NULL )
{}




// -----------------------------------------------------------------------------
// Constructor with a convex and the crust thickness
__host__ __device__ 
RigidBody::RigidBody( Convex* convex, 
                      double ct )
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
__host__ __device__
RigidBody::~RigidBody()
{
    delete m_convex;
    delete m_boundingBox;
}




// -----------------------------------------------------------------------------
// Returns the rigid body's convex
__host__ __device__
Convex* RigidBody::getConvex() const
{
    return ( m_convex );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's crust thickness
__host__ __device__
double RigidBody::getCrustThickness() const
{
    return ( m_crustThickness );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's volume
__host__ __device__
double RigidBody::getVolume() const
{
    return ( m_volume );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's inertia
__host__ __device__
double* RigidBody::getInertia() const
{
    return ( m_inertia );
}




// -----------------------------------------------------------------------------
// Returns the inverse of rigid body's inertia
__host__ __device__
double* RigidBody::getInertia_1() const
{
    return ( m_inertia_1 );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's bounding box
__host__ __device__
BoundingBox* RigidBody::getBoundingBox() const
{
    return ( m_boundingBox );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's circumscribed radius
__host__ __device__
float RigidBody::getCircumscribedRadius() const
{
    return ( m_circumscribedRadius );
}




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Returns whether 2 rigid bodies intersect
__host__ __device__
bool intersectRigidBodies( RigidBody const& rbA,
                           RigidBody const& rbB,
                           Transform3d const& a2w,
                           Transform3d const& b2w )
{
    Convex const* convexA = rbA.getConvex();
    Convex const* convexB = rbB.getConvex();

    // In case the 2 rigid bodies are spheres
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        double radiiSum = (double) ( rbA.getCircumscribedRadius() + 
                                     rbB.getCircumscribedRadius() );
        double dist2 = ( a2w.getOrigin() - b2w.getOrigin() ).norm2();
        return ( dist2 < radiiSum * radiiSum );
    }

    // General case
    Vec3d temp = a2w.getOrigin();
    Vec3f cenA( ( float ) temp[X], ( float ) temp[Y], ( float ) temp[Z] );
    temp = b2w.getOrigin();
    Vec3f cenB( ( float ) temp[X], ( float ) temp[Y], ( float ) temp[Z] );
    float dist = ( cenB - cenA ).norm();
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
__host__ __device__
bool intersectRigidBodies( RigidBody const& rbA,
                           RigidBody const& rbB,
                           Transform3d const& b2a )
{
    Convex const* convexA = rbA.getConvex();
    Convex const* convexB = rbB.getConvex();

    // In case the 2 rigid bodies are spheres
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        double radiiSum = (double) ( rbA.getCircumscribedRadius() + 
                                     rbB.getCircumscribedRadius() );
        double dist2 = ( b2a.getOrigin() ).norm2();
        return ( dist2 < radiiSum * radiiSum );
    }

    // General case
    Vec3d temp = b2a.getOrigin();
    Vec3f posB2A( ( float ) temp[X], ( float ) temp[Y], ( float ) temp[Z] );
    float dist = posB2A.norm();
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
__host__ __device__
ContactInfoD closestPointsRigidBodies( RigidBody const& rbA,
                                       RigidBody const& rbB,
                                       Transform3d const& a2w,
                                       Transform3d const& b2w )
{
    Convex const* convexA = rbA.getConvex();
    Convex const* convexB = rbB.getConvex();

    // General case
    Vec3f cenA = a2w.getOrigin();
    Vec3f cenB = b2w.getOrigin();
    float dist = ( cenB - cenA ).norm();
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
            Vec3d ptA, ptB;
            // double distance = closestPointsGJK( *convexA, 
            //                                     *convexB,
            //                                     a2w,
            //                                     b2w,
            //                                     ptA,
            //                                     ptB,
            //                                     nbIterGJK );
            double distance = compute_minimum_distance( *convexA, 
                                                *convexB,
                                                a2w,
                                                b2w,
                                                ptA,
                                                ptB,
                                                nbIterGJK );
            // TODO: ERROR HANDLING
            
            // Sum of crust thicknesses
            double ctSum = rbA.getCrustThickness() + rbB.getCrustThickness();

            // Points A and B are in their respective local coordinate systems
            // Thus we transform them into the world coordinate system
            ptA = (a2w)( ptA );
            ptB = (b2w)( ptB );

            // Contact point definition as the mid point between ptA and ptB
            Vec3d contactPt = ( ptA + ptB ) / 2.;

            // Computation of the actual overlap vector
            // If contact, crustA + crustB - distance > 0, the overlap vector is
            // directed from B to A
            // If no contact, crustA + crustB - distance < 0 and we do not care 
            // about the direction of the overlap vector
            Vec3d contactVec = ( ptA - ptB ) / distance;
            contactVec.round();
            contactVec *= ctSum - distance;

            // Computation of the actual overlap 
            // distance = distance - crustA - crustB
            // If actual overlap distance < 0 => contact otherwise no contact
            distance -= ctSum;

            return ( ContactInfoD( contactPt, contactVec, distance ) );
        }
        else
            return ( ContactInfoD( Vec3d( 0., 0., 0. ), 
                                   Vec3d( 0., 0., 0. ),
                                   1.e20 ) );
    }
    else
        return ( ContactInfoD( Vec3d( 0., 0., 0. ), 
                               Vec3d( 0., 0., 0. ),
                               1.e20 ) );
}