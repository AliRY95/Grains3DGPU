#include "AABB.hh"


// -----------------------------------------------------------------------------
// Constructor
__host__ __device__
AABB::AABB()
{}




// -----------------------------------------------------------------------------
// Constructor with half edge length as input parameters
__host__ __device__
AABB::AABB( float x, float y, float z )
: m_extent( Vec3f( x, y, z ) )
{}




// -----------------------------------------------------------------------------
// Constructor with a vector containing the edge half-lengths
__host__ __device__
AABB::AABB( Vec3f const& extent_ )
: m_extent( extent_ )
{}




// -----------------------------------------------------------------------------
// Destructor
__host__ __device__
AABB::~AABB()
{}




// -----------------------------------------------------------------------------
// Sets values of the edge length
__host__ __device__
void AABB::setExtent( float x, float y, float z )
{
    m_extent = Vec3f( x, y, z );
}




// -----------------------------------------------------------------------------
// Sets extent values using ccording to the convex which we want to fit the AABB
// around.
__host__ __device__
void AABB::setExtent( Convex const& convex )
{
    m_extent = convex.computeAABB();
}




// -----------------------------------------------------------------------------
// Gets values of the edge length
__host__ __device__
Vec3f const AABB::getExtent() const
{
    return ( m_extent );
}






/* ========================================================================== */
/*                              External Methods                              */
/* ========================================================================== */
// Returns whether the AABBs are in contact
__host__ __device__
bool intersectAABB( AABB const& a, 
                    AABB const& b,
                    Vec3f const& posA,
                    Vec3f const& posB )
{
    Vec3f const lenA = a.getExtent();
    Vec3f const lenB = b.getExtent();
    if      ( fabsf( posA[X] - posB[X] ) > ( lenA[X] + lenB[X] ) )
        return ( false );
    else if ( fabsf( posA[Y] - posB[Y] ) > ( lenA[Y] + lenB[Y] ) )
        return ( false );
    else if ( fabsf( posA[Z] - posB[Z] ) > ( lenA[Z] + lenB[Z] ) )
        return ( false );
    else // overlap
        return ( true );
}




// -----------------------------------------------------------------------------
// Returns whether the AABBs are in contact - relative position
__host__ __device__
bool intersectAABB( AABB const& a, 
                    AABB const& b,
                    Vec3f const& posB2A )
{
    Vec3f const lenA = a.getExtent();
    Vec3f const lenB = b.getExtent();
    if      ( fabsf( posB2A[X] ) > ( lenA[X] + lenB[X] ) )
        return ( false );
    else if ( fabsf( posB2A[Y] ) > ( lenA[Y] + lenB[Y] ) )
        return ( false );
    else if ( fabsf( posB2A[Z] ) > ( lenA[Z] + lenB[Z] ) )
        return ( false );
    else // overlap
        return ( true );
}