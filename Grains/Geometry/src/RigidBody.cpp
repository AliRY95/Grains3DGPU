#include "Convex.hh"
#include "AABB.hh"
#include "RigidBody.hh"


// -----------------------------------------------------------------------------
// Default constructor
__host__ __device__ RigidBody::RigidBody()
: m_convex( NULL )
, m_crustThickness( 0. )
, m_AABB( NULL )
{}




// -----------------------------------------------------------------------------
// Constructor with a convex and the crust thickness
__host__ __device__ RigidBody::RigidBody( Convex* convex, 
                                          double ct )
: m_convex( convex )
, m_crustThickness( ct )
{
    m_volume = m_convex->computeVolume();
    // bool tmp = m_convex->computeInertia( m_inertia, m_inertia_1 );
    m_AABB = new AABB( m_convex->computeAABB() );
    m_circumscribedRadius = m_convex->computeCircumscribedRadius();
}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__ RigidBody::~RigidBody()
{
    delete m_convex;
    delete m_AABB;
}




// -----------------------------------------------------------------------------
// Returns the rigid body's convex
__host__ __device__ Convex* RigidBody::getConvex() const
{
    return ( m_convex );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's crust thickness
__host__ __device__ double RigidBody::getCrustThickness() const
{
    return ( m_crustThickness );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's volume
__host__ __device__ double RigidBody::getVolume() const
{
    return ( m_volume );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's inertia
__host__ __device__ double* RigidBody::getInertia() const
{
    return ( m_inertia );
}




// -----------------------------------------------------------------------------
// Returns the inverse of rigid body's inertia
__host__ __device__ double* RigidBody::getInertia_1() const
{
    return ( m_inertia_1 );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's AABB
__host__ __device__ AABB* RigidBody::getAABB() const
{
    return ( m_AABB );
}




// -----------------------------------------------------------------------------
// Returns the rigid body's circumscribed radius
__host__ __device__ float RigidBody::getCircumscribedRadius() const
{
    return ( m_circumscribedRadius );
}




/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Returns whether 2 rigid bodies intersect
__host__ __device__ bool intersectRigidBodies( RigidBody const& rbA,
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
        double radiiSum = (double) rbA.getCircumscribedRadius() + 
                                   rbB.getCircumscribedRadius();
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
        // if( intersectAABB( *( rbA.getAABB() ), *( rbB.getAABB() ), cenA, cenB ) )
            return( intersectGJK( *convexA, *convexB, a2w, b2w ) );
    }
    return ( false );
}




// -----------------------------------------------------------------------------
// Returns whether 2 rigid bodies intersect
__host__ __device__ bool intersectRigidBodies( RigidBody const& rbA,
                                               RigidBody const& rbB,
                                               Transform3d const& b2a )
{
    Convex const* convexA = rbA.getConvex();
    Convex const* convexB = rbB.getConvex();

    // In case the 2 rigid bodies are spheres
    if ( convexA->getConvexType() == SPHERE && 
         convexB->getConvexType() == SPHERE )
    {
        double radiiSum = (double) rbA.getCircumscribedRadius() + 
                                   rbB.getCircumscribedRadius();
        double dist2 = ( b2a.getOrigin() ).norm2();
        return ( dist2 < radiiSum * radiiSum );
    }

    // General case
    Vec3d temp = b2a.getOrigin();
    Vec3f posB2A( ( float ) temp[X], ( float ) temp[Y], ( float ) temp[Z] );
    float dist = posB2A.norm();
    if ( dist < rbA.getCircumscribedRadius() + rbB.getCircumscribedRadius() )
    {
        // if( intersectAABB( *( rbA.getAABB() ), *( rbB.getAABB() ), posB2A ) )
            return( intersectGJK( *convexA, *convexB, b2a ) );
    }
    return ( false );
}